from pyulog import ULog
import glob,numpy as np,json,os
out=[]
for f in sorted(glob.glob('FlightLogs/2026-09-25/*.ulg')):
    u=ULog(f); D={}
    for d in u.data_list:
        if d.multi_id==0: D.setdefault(d.name,d.data)
    if 'vehicle_status' not in D or not (D['vehicle_status']['nav_state']==14).any(): continue
    st=D['vehicle_status']; ts=st['timestamp']/1e6; to0=ts[np.where(st['nav_state']==14)[0][0]]
    nxt=np.where((st['nav_state']!=14)&(ts>to0))[0]; to1=ts[nxt[0]] if len(nxt) else ts[-1]
    lk=D['actuator_armed']; tl=lk['timestamp']/1e6; kk=np.where((lk['manual_lockdown']==1)&(tl>=to0))[0]; tk=tl[kk[0]] if len(kk) else to1
    ar=np.where((lk['armed']==1))[0]; tarm=tl[ar[0]]
    r={'f':f[-12:-4],'to0':to0-tarm,'to1':to1-tarm,'tk':tk-tarm}
    # hover thrust vs V: thrust cmd in first 2 s of offboard (hover)
    th=D['vehicle_thrust_setpoint']; tt=th['timestamp']/1e6; z=-th['xyz[2]']
    w=(tt>=to0+0.5)&(tt<=to0+2.5); r['thr_hover_cmd']=float(np.median(z[w])) if w.any() else None
    he=D['hover_thrust_estimate']; th_e=he['timestamp']/1e6
    w2=(th_e<=to0)&(th_e>=to0-6); r['hover_est_pre']=float(np.median(he['hover_thrust'][w2])) if w2.any() else float(np.median(he['hover_thrust'][:20]))
    b=D['battery_status']; tb=b['timestamp']/1e6; r['V_hover']=float(np.median(b['voltage_v'][(tb>=to0)&(tb<=to0+2.5)]))
    # vertical velocity during hover phase before descent (drift): 
    lp=D['vehicle_local_position']; tp=lp['timestamp']/1e6
    # precision: xy at arm (first sample after arm) vs at offboard end & at contact/kill
    def xy(t): i=np.searchsorted(tp,t); i=min(i,len(tp)-1); return np.array([lp['x'][i],lp['y'][i]])
    x0=xy(tarm+0.5); r['xy_end_off']=float(np.linalg.norm(xy(to1)-x0)); r['xy_kill']=float(np.linalg.norm(xy(tk)-x0))
    r['xy_hover']=float(np.linalg.norm(xy(to0+1.5)-x0))
    # contact speed: vz over 0.15 s before kill/landing min-agl; use full window to kill
    ds=D['distance_sensor']; td=ds['timestamp']/1e6; agl=ds['current_distance']
    w3=(tp>=to0)&(tp<=tk); tw=tp[w3]; ag=np.interp(tw,td,agl); vz=lp['vz'][w3]
    k2=np.where(ag>0.15)[0]; r['vz_last_agl>0.15']=float(vz[k2[-1]]) if len(k2) else None
    r['t_off_end_to_contact']=float(tw[np.argmin(ag)]-to1) if len(tw) else None
    r['vz_at_handoff']=float(np.interp(to1,tp,lp['vz'])); r['agl_at_handoff']=float(np.interp(to1,td,agl))
    # rate loop lag
    rs=D['vehicle_rates_setpoint']; av=D['vehicle_angular_velocity']; tr=rs['timestamp']/1e6; ta=av['timestamp']/1e6
    w4=(tr>=to0)&(tr<=to1); tg=np.arange(to0,to1,0.004)
    lags=[]
    for ax,sk,mk in [(0,'roll','xyz[0]'),(1,'pitch','xyz[1]')]:
        sp=np.interp(tg,tr[w4],rs[sk][w4]) if sk in rs else None
        if sp is None: continue
        mv=np.interp(tg,ta,av[mk])
        best=None
        for lag in range(0,60):     # 0..240 ms
            a=sp[:len(sp)-lag] if lag else sp; b=mv[lag:]
            c=np.corrcoef(a,b)[0,1]
            if best is None or c>best[1]: best=(lag*4,c)
        lags.append(best)
    r['rate_lag_ms']=[l[0] for l in lags]; r['rate_corr']=[round(l[1],2) for l in lags]
    # saturation: motors >0.95 or thrust setpoint >0.9 in last 2 s of offboard
    am=D['actuator_motors']; tm=am['timestamp']/1e6; w5=(tm>=to1-2)&(tm<=to1)
    mm=np.stack([am['control[%d]'%i] for i in range(4)],1)[w5]
    r['motor_max_frac95']=float((mm.max(1)>0.95).mean()) if len(mm) else None
    r['motor_min_lastcs']=float(mm.min()) if len(mm) else None
    w6=(tt>=to1-2)&(tt<=to1); r['thr_max_last2s']=float(z[w6].max()) if w6.any() else None
    # attitude-setpoint tilt cap
    out.append(r)
json.dump(out,open(os.environ['TEMP']+'/sp/ulg2.json','w'),default=float)
import statistics as S
def col(k): return [r[k] for r in out if r.get(k) is not None]
print('n',len(out))
print('hover thrust cmd vs est vs V: corr(V,hover_cmd)=%.2f corr(V,hover_est)=%.2f'%(np.corrcoef(col('V_hover'),col('thr_hover_cmd'))[0,1],np.corrcoef(col('V_hover'),col('hover_est_pre'))[0,1]))
Vh=np.array(col('V_hover'));hc=np.array(col('thr_hover_cmd'));he_=np.array(col('hover_est_pre'))
print('slope hover_cmd vs V: %.4f /V ; hover_est %.4f /V ; cmd/est ratio med %.3f range %.2f-%.2f'%(np.polyfit(Vh,hc,1)[0],np.polyfit(Vh,he_,1)[0],np.median(hc/he_),(hc/he_).min(),(hc/he_).max()))
for k in ['xy_hover','xy_end_off','xy_kill','agl_at_handoff','vz_at_handoff','vz_last_agl>0.15','t_off_end_to_contact','motor_max_frac95','thr_max_last2s']:
    v=np.array(col(k)); print('%-22s med %.2f p10 %.2f p90 %.2f max %.2f'%(k,np.median(v),np.percentile(v,10),np.percentile(v,90),v.max()))
lg=np.array([r['rate_lag_ms'] for r in out if r.get('rate_lag_ms')]); print('rate-loop lag ms roll/pitch median',np.median(lg,0),'corr',np.median([r['rate_corr'] for r in out if r.get('rate_corr')],0))

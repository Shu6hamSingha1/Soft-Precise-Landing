from pyulog import ULog
import glob,numpy as np,json,sys
def T(u,name,multi=0):
    for d in u.data_list:
        if d.name==name and d.multi_id==multi: return d.data
    return None
def yawf(q): return np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2))
out=[]
for f in sorted(glob.glob('FlightLogs/2026-09-25/*.ulg')):
    u=ULog(f); r={'file':f[-12:-4]}
    st=T(u,'vehicle_status'); ts=st['timestamp']/1e6; nav=st['nav_state']; arm=st['arming_state']
    t0=ts[0]
    r['dur']=(u.last_timestamp-u.start_timestamp)/1e6
    off=np.where(nav==14)[0]
    if len(off)==0: r['off']=None; out.append(r); continue
    to0=ts[off[0]]; 
    # offboard end = first ts after to0 with nav!=14
    nxt=np.where((nav!=14)&(ts>to0))[0]; to1=ts[nxt[0]] if len(nxt) else ts[-1]
    lock=T(u,'actuator_armed'); tl=lock['timestamp']/1e6; ml=lock['manual_lockdown']
    kk=np.where((ml==1)&(tl>=to0))[0]; tk=tl[kk[0]] if len(kk) else None
    msgs=[(m.timestamp/1e6,m.message.strip()) for m in u.logged_messages]
    pt=[t for t,m in msgs if 'Pilot took over' in m and t>=to0]
    r.update(off_start=to0-t0,off_end=to1-t0,off_dur=to1-to0,kill=None if tk is None else tk-t0,pilot=[round(x-t0,1) for x in pt][:2],
             lowbat=any('Low battery' in m for t,m in msgs),msgs=[m for t,m in msgs if 'Failsafe' in m or 'blind' in m or 'Low battery' in m][:3])
    end=min([x for x in [to1,tk] if x is not None])
    b=T(u,'battery_status'); tb=b['timestamp']/1e6; v=b['voltage_v']; 
    r['V_off']=float(np.interp(to0,tb,v)); r['V_min']=float(v[(tb>=to0)&(tb<=end)].min()); r['V_first']=float(v[0]); r['I_max']=float(b['current_a'][(tb>=to0)&(tb<=end)].max())
    h=T(u,'hover_thrust_estimate'); 
    if h is not None: r['hover_thr']=float(np.median(h['hover_thrust'][(h['timestamp']/1e6>=to0)&(h['timestamp']/1e6<=end)]))
    lp=T(u,'vehicle_local_position'); tp=lp['timestamp']/1e6
    w=(tp>=to0)&(tp<=end)
    z=lp['z'][w]; r['z_start']=float(-z[0]); r['z_end']=float(-z[-1]); r['zmax']=float(-z.min())
    vx,vy,vz=lp['vx'][w],lp['vy'][w],lp['vz'][w]; tw=tp[w]
    ds=T(u,'distance_sensor'); 
    if ds is not None:
        td=ds['timestamp']/1e6; agl=np.interp(tw,td,ds['current_distance'])
    else: agl=-z
    r['agl_end']=float(agl[-1]); 
    m=(agl<1.0)&(agl>0.4); r['vz_1to04']=float(np.median(vz[m])) if m.any() else None
    m2=(agl<2.0)&(agl>1.0); r['vz_2to1']=float(np.median(vz[m2])) if m2.any() else None
    m3=(agl<0.4)&(agl>0.15); r['vz_low']=float(np.median(vz[m3])) if m3.any() else None
    r['vz_max_below1']=float(vz[agl<1.0].max()) if (agl<1.0).any() else None
    # contact: vz at last sample with agl>0.2
    k=np.where(agl>0.2)[0]; 
    r['vz_contact']=float(np.mean(vz[max(k[-1]-3,0):k[-1]+1])) if len(k) else None
    lat=np.hypot(vx,vy); l2=agl<2.0
    r['lat_rms_below2']=float(np.sqrt(np.mean(lat[l2]**2))) if l2.any() else None
    r['lat_max_below2']=float(lat[l2].max()) if l2.any() else None
    r['xy_disp_below2']=float(np.hypot(np.ptp(lp['x'][w][l2]),np.ptp(lp['y'][w][l2]))) if l2.any() else None
    at=T(u,'vehicle_attitude'); ta=at['timestamp']/1e6; q=np.stack([at['q[0]'],at['q[1]'],at['q[2]'],at['q[3]']],1)
    wa=(ta>=to0)&(ta<=end); q=q[wa]; tt=ta[wa]
    roll=np.degrees(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2))); pit=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,2]-q[:,3]*q[:,1]),-1,1)))
    tilt=np.hypot(roll,pit); aglA=np.interp(tt,tw,agl)
    r['tilt_max']=float(tilt.max()); r['tilt_p95']=float(np.percentile(tilt,95))
    lw=aglA<2.0
    r['tilt_max_below2']=float(tilt[lw].max()) if lw.any() else None
    yw=np.unwrap(yawf(q)); r['yaw_drift']=float(np.degrees(yw[-1]-yw[0])); r['yaw_ptp']=float(np.degrees(np.ptp(yw)))
    av=T(u,'vehicle_angular_velocity'); tv=av['timestamp']/1e6; wv=(tv>=to0)&(tv<=end)
    r['rate_max']=float(np.degrees(np.max(np.abs(np.stack([av['xyz[0]'],av['xyz[1]']],1)[wv]))))
    r['yawrate_max']=float(np.degrees(np.max(np.abs(av['xyz[2]'][wv]))))
    th=T(u,'vehicle_thrust_setpoint'); tt2=th['timestamp']/1e6; wt=(tt2>=to0)&(tt2<=end)
    r['thr_z_min']=float(-th['xyz[2]'][wt].max()) if wt.any() else None
    r['thr_mean_last2s']=float(np.mean(-th['xyz[2]'][wt][-100:]))
    rc=T(u,'input_rc'); 
    out.append(r)
json.dump(out,open(sys.argv[1],'w'),default=float)
print(len(out))

from pyulog import ULog
import glob,numpy as np,json
res=[]
for f in sorted(glob.glob('FlightLogs/2026-09-25/*.ulg')):
    u=ULog(f); D={}
    for d in u.data_list:
        if d.multi_id==0: D.setdefault(d.name,d.data)
    if 'vehicle_status' not in D or not (D['vehicle_status']['nav_state']==14).any(): continue
    st=D['vehicle_status']; ts=st['timestamp']/1e6; to0=ts[np.where(st['nav_state']==14)[0][0]]
    lock=D['actuator_armed']; tl=lock['timestamp']/1e6; kk=np.where((lock['manual_lockdown']==1)&(tl>=to0))[0]
    tk=tl[kk[0]] if len(kk) else 1e9
    sc=D['sensor_combined']; t=(sc['timestamp']/1e6)
    a=np.stack([sc['accelerometer_m_s2[0]'],sc['accelerometer_m_s2[1]'],sc['accelerometer_m_s2[2]']],1)
    g=np.stack([sc['gyro_rad[0]'],sc['gyro_rad[1]'],sc['gyro_rad[2]']],1)
    ds=D['distance_sensor']; agl=np.interp(t,ds['timestamp']/1e6,ds['current_distance'])
    lp=D['vehicle_local_position']; vz=np.interp(t,lp['timestamp']/1e6,lp['vz']); 
    w=(t>=to0)&(t<=tk)
    # descent flight: after peak alt
    ip=np.argmax(np.where(w,agl,0))
    w2=w&(np.arange(len(t))>ip)
    an=np.linalg.norm(a,axis=1); az=a[:,2]
    ld=D['vehicle_land_detected']; tld=ld['timestamp']/1e6; landed=ld['landed']
    tland=tld[np.where((landed==1)&(tld>to0))[0][0]] if ((landed==1)&(tld>to0)).any() else None
    # ground-truth contact: last time agl>0.3 in descent before min
    idx=np.where(w2)[0]; jmin=idx[np.argmin(agl[idx])]
    tc=t[jmin] if agl[jmin]<0.5 else None
    res.append(dict(f=f[-12:-4],t=t.tolist(),an=an.tolist(),az=az.tolist(),agl=agl.tolist(),vz=vz.tolist(),w2=w2.tolist(),tk=tk if tk<1e8 else None,tland=tland,tc=tc,to0=to0))
    print(f[-12:-4],'tc=',None if tc is None else round(tc-to0,2),'tland=',None if tland is None else round(tland-to0,2),'tk=',None if tk>1e8 else round(tk-to0,2),'agl_min=%.2f'%agl[jmin], 'an: pre(agl>0.6) p99=%.1f max=%.1f | last1s max=%.1f'%(np.percentile(an[w2&(agl>0.6)],99),an[w2&(agl>0.6)].max(),an[w2&(t>=t[jmin]-1)&(t<=t[jmin]+0.5)].max()))
import pickle; pickle.dump(res,open('%s/sp/imu.pkl'%__import__('os').environ['TEMP'],'wb'))

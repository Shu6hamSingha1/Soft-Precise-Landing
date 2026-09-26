import numpy as np,glob,json
from pyulog import ULog
def feat(t,agl):
    # last descending crossing of 2.0m, then time to min agl afterwards; and peak agl
    t=np.asarray(t);a=np.asarray(agl)
    i=np.where((a[:-1]>=2.0)&(a[1:]<2.0))[0]
    if len(i)==0: return None
    i=i[-1]; j=i+np.argmin(a[i:]);
    k=np.where((a[:-1]>=1.0)&(a[1:]<1.0))[0]; k=k[k>=i]
    return dict(t21=(t[k[0]]-t[i]) if len(k) else None, t2end=t[j]-t[i], amin=float(a[j]), peak=float(a.max()))
def L(f):
    x=np.load(f,allow_pickle=True); return x.item() if x.shape==() else x
pi=[]
for d in sorted(glob.glob('Landing/2026-09-25/Fri*')):
    try: c=L(d+'/Control_Data.npy'); T=L(d+'/Telemetry_Data.npy')
    except: continue
    if len(c['t'])<20: continue
    t=np.array(T['Distance Sensor Timestamp']); a=np.array([x.current_distance_m for x in T['Distance Sensor']])
    f=feat(t,a); pi.append((d[-13:-5],f))
ul=[]
for f in sorted(glob.glob('FlightLogs/2026-09-25/*.ulg')):
    u=ULog(f)
    if not any(d.name=='vehicle_status' for d in u.data_list): continue
    st=[d for d in u.data_list if d.name=='vehicle_status'][0].data
    if not (st['nav_state']==14).any(): continue
    ds=[d for d in u.data_list if d.name=='distance_sensor'][0].data
    ul.append((f[-12:-4],feat(ds['timestamp']/1e6,ds['current_distance'])))
print(len(pi),len(ul))
json.dump({'pi':pi,'ul':ul},open('../../../../../../../tmp_match.json','w')) if False else None
for i in range(max(len(pi),len(ul))):
    a=pi[i] if i<len(pi) else ('',None); b=ul[i] if i<len(ul) else ('',None)
    g=lambda x:'%5.1f/%5.1f/%4.2f/%4.1f'%(x['t21'] or 0,x['t2end'],x['amin'],x['peak']) if x else '   none'
    print(i,a[0],g(a[1]),'|',b[0],g(b[1]))

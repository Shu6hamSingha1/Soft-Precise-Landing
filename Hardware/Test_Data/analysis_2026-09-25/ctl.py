import numpy as np,glob,json
def L(f):
    x=np.load(f,allow_pickle=True); return x.item() if x.shape==() else x
def A(v,n): return np.array([np.asarray(x,float) for x in list(v)[:n]])
out=[]
for d in sorted(glob.glob('Fri*')):
    c=L(d+'/Control_Data.npy'); I=L(d+'/Img_Data.npy')
    if len(c['t'])<20: continue
    n=min(len(c[k]) for k in ['t','e_a(t)','a_u(t)','I_a(t)','sigma(t)','kappa(t)','s_e_n(t)','izeta(t)','ie_a(t)','p_s(t)','theta_ctrl(t)','MARKER_EXTENT_PX(t)','e_R(t)'])
    t=np.array(c['t'][:n]); r={'run':d[-13:-5],'n':n,'dur':float(t[-1]-t[0]),'loop_dt_med':float(np.median(np.diff(t))*1e3),'loop_dt_p99':float(np.percentile(np.diff(t),99)*1e3)}
    ea=np.array(c['e_a(t)'][:n],float); r['ea0']=float(ea[0]);r['ea_end']=float(ea[-1]);r['ea_absmax']=float(np.abs(ea).max());r['ea_conv']=bool(abs(ea[-1])<abs(ea[0]))
    r['ea_grow']=float(abs(ea[-1])/max(abs(ea[0]),1e-3))
    sen=A(c['s_e_n(t)'],n); m=np.abs(sen).max(1)
    r['sen_max']=float(m.max()); r['sen_gt1_frac']=float((m>1).mean()); r['sen_end']=float(m[-1])
    r['sen_first_gt1_t']=float(t[np.argmax(m>1)]-t[0]) if (m>1).any() else None
    k=A(c['kappa(t)'],n); r['kap_max']=k.max(0).tolist(); r['kap_clamp_frac']=float((k[:,0]>=29.9).mean())
    au=A(c['a_u(t)'],n); r['au_xy_max']=float(np.abs(au[:,:2]).max()); r['au_xy_p99']=float(np.percentile(np.abs(au[:,:2]),99)); r['au_xy_gt10']=int((np.abs(au[:,:2]).max(1)>10).sum())
    r['au_z_min']=float(au[:,2].min()); r['au_z_max']=float(au[:,2].max())
    Ia=A(c['I_a(t)'],n); r['Ia_xy_max']=float(np.abs(Ia[:,:2]).max()); r['Ia_z_min']=float(Ia[:,2].min()); r['Ia_z_max']=float(Ia[:,2].max())
    iz=A(c['izeta(t)'],n); r['izeta_max']=float(np.abs(iz).max()); r['ie_a_max']=float(np.abs(np.array(c['ie_a(t)'][:n],float)).max())
    ex=np.array(c['MARKER_EXTENT_PX(t)'][:n],float); r['extent_nz']=float((ex>0).mean())
    th=A(c['theta_ctrl(t)'],n); r['theta_max']=float(th.max())
    eR=A(c['e_R(t)'],n); r['eR_xy_max']=float(np.abs(eR[:,:2]).max()); r['eR_z_end']=float(eR[-1,2])
    fps=np.array(I['FPS'],float); r['fps_med']=float(np.nanmedian(fps[fps>0])) if (fps>0).any() else None
    ct=np.array([x['pulled_at_perf_counter'] for x in I['Capture Stamp']],float); tt=np.array(I['Time'],float)
    if len(ct)==len(tt) and len(tt)>5: r['img_lat_ms_med']=float(np.median(tt-ct)*1e3)
    tag=np.array(I['S Estimator Tag']); 
    r['S_tags']={str(u):int((tag==u).sum()) for u in np.unique(tag)}
    pc=np.array(I['Planar Map Confidence'],float); r['pm_conf_med']=float(np.nanmedian(pc)) if len(pc) else None
    out.append(r)
json.dump(out,open('%s/sp/ctl.json'%__import__('os').environ['TEMP'],'w'),default=float)
print(len(out))

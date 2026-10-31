import json, math, sys
rows=[json.loads(l) for l in open(sys.argv[1])]
bi=[i for i,r in enumerate(rows) if r['ev']=='boot'][0]
R=rows[bi:]
def enu(lat,lon,h):
    return ((lon-h[1])*111320*math.cos(math.radians(h[0])), (lat-h[0])*110574)
flights=[('F0 hop',1664004,1724135,(9.0271121,-79.5310134)),('F1',1975087,2475513,(9.0271049,-79.5309978)),
         ('F2',2502592,3435263,(9.0271407,-79.5309879)),('F3',3437261,3594982,(9.0271430,-79.5309888))]
for name,t0,t1,home in flights:
    print('=====',name, 'takeoff',t0/1000,'landed-event',t1/1000)
    prev=None
    for r in R:
        if not (t0-15000<=r['t']<=t1+5000): continue
        if r['ev']=='tlm':
            if not r['gps']: continue
            e,n=enu(r['lat'],r['lon'],home)
            sp=''
            if prev and r['t']-prev[0]<=1500:
                dt=(r['t']-prev[0])/1000; sp='%4.1f'%(math.hypot(e-prev[1],n-prev[2])/dt)
            print('%8.1f  E%6.1f N%6.1f alt%5.1f spd%s dA%d st%d'%(r['t']/1000,e,n,r['alt'],sp,r['dA'],r['st']))
            prev=(r['t'],e,n)
        elif r['ev'] in('osdk_mission',):
            print('%8.1f  ** %s wp=%s st=%s b2=%s'%(r['t']/1000,'STATUS' if r['kind']=='status' else 'EVENT ',r['wp'],r['st'],r['b2']))
        else:
            ex={k:v for k,v in r.items() if k not in('t','ev')}
            if r['ev']=='drop':
                e,n=enu(r['lat'],r['lon'],home); ex='door %d at E%.1f N%.1f'%(r['door'],e,n)
            print('%8.1f  ## %s %s'%(r['t']/1000,r['ev'],ex))

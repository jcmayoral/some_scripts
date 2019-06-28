""" Original Author: Haoqiang Fan """
import numpy as np
import ctypes as ct
import cv2
import sys
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
showsz=800
mousex,mousey=0.5,0.5
zoom=1.0
changed=True
def onmouse(*args):
    global mousex,mousey,changed
    y=args[1]
    x=args[2]
    mousex=x/float(showsz)
    mousey=y/float(showsz)
    changed=True
cv2.namedWindow('show3d')
cv2.moveWindow('show3d',0,0)
cv2.setMouseCallback('show3d',onmouse)



class PointInfo:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0


def render_ball(h, w, show, n , xyzs_list, c0, c1, c2, r):
#void render_ball(int h,int w,unsigned char * show,int n,int * xyzs,float * c0,float * c1,float * c2,int r){
    print r
    r=np.maximum(r,1);
    #vector<int> depth(h*w,-2100000000);
    depth = np.zeros(h*w)

    pattern = list()
    #vector<PointInfo> pattern;
    for dx in range(-r,r):
        for dy in range(-r,r):
            if (dx*dx+dy*dy<r*r):
                dz=np.sqrt(r*r-dx*dx-dy*dy)
                pinfo = PointInfo()
                pinfo.x=dx
                pinfo.y=dy
                pinfo.z=dz
                pinfo.r=dz/r
                pinfo.g=dz/r
                pinfo.b=dz/r
                pattern.append(pinfo)
    zmin = 0
    zmax = 0
    for i in range(0,n):
        if i==0:
            zmin=xyzs_list[i*3][2]-r;
            zmax=xyzs_list[i*3][2]+r;
        else:
            zmin=np.minimum(zmin,xyzs_list[i][2]-r)
            zmax=np.maximum(zmax,xyzs_list[i][2]+r)

    for i in range(0,n):
        x=xyzs_list[i][0]
        y=xyzs_list[i][1]
        z=xyzs_list[i][2]
        for j in range(0,len(pattern)):
            x2=x+pattern[j].x
            y2=y+pattern[j].y;
            z2=z+pattern[j].z;

            if not(x2<0 or x2>=h or y2<0 or y2>=w) and depth[x2*w+y2]<z2:
                depth[x2*w+y2]=z2;
                intensity=np.minimum(1.0,(z2-zmin)/(zmax-zmin)*0.7+0.3)
                show[x2][y2][0]=pattern[j].b*c2[i]*intensity
                show[x2][y2][1]=pattern[j].g*c0[i]*intensity
                show[x2][y2][2]=pattern[j].r*c1[i]*intensity
#dll=np.ctypeslib.load_library(os.path.join(BASE_DIR, 'render_balls_so'),'.')

def showpoints(xyz,c_gt=None, c_pred = None ,waittime=0,showrot=False,magnifyBlue=0,freezerot=False,background=(0,0,0),normalizecolor=True,ballradius=10):
    global showsz,mousex,mousey,zoom,changed
    xyz=xyz-xyz.mean(axis=0)
    radius=((xyz**2).sum(axis=-1)**0.5).max()
    xyz/=(radius*2.2)/showsz
    if c_gt is None:
        c0=np.zeros((len(xyz),),dtype='float32')+255
        c1=np.zeros((len(xyz),),dtype='float32')+255
        c2=np.zeros((len(xyz),),dtype='float32')+255
    else:
        c0=c_gt[:,0]
        c1=c_gt[:,1]
        c2=c_gt[:,2]


    if normalizecolor:
        c0/=(c0.max()+1e-14)/255.0
        c1/=(c1.max()+1e-14)/255.0
        c2/=(c2.max()+1e-14)/255.0


    c0=np.require(c0,'float32','C')
    c1=np.require(c1,'float32','C')
    c2=np.require(c2,'float32','C')

    show=np.zeros((showsz,showsz,3),dtype='uint8')

    def render():
        rotmat=np.eye(3)
        if not freezerot:
            xangle=(mousey-0.5)*np.pi*1.2
        else:
            xangle=0
        rotmat=rotmat.dot(np.array([
            [1.0,0.0,0.0],
            [0.0,np.cos(xangle),-np.sin(xangle)],
            [0.0,np.sin(xangle),np.cos(xangle)],
            ]))
        if not freezerot:
            yangle=(mousex-0.5)*np.pi*1.2
        else:
            yangle=0
        rotmat=rotmat.dot(np.array([
            [np.cos(yangle),0.0,-np.sin(yangle)],
            [0.0,1.0,0.0],
            [np.sin(yangle),0.0,np.cos(yangle)],
            ]))
        rotmat*=zoom
        nxyz=xyz.dot(rotmat)+[showsz/2,showsz/2,0]
        ixyz=nxyz.astype('int32')
        show[:]=background

        render_ball(
            show.shape[0],
            show.shape[1],
            show,
            ixyz.shape[0],
            ixyz,
            c0,
            c1,
            c2,
            ballradius
        )

        if magnifyBlue>0:
            show[:,:,0]=np.maximum(show[:,:,0],np.roll(show[:,:,0],1,axis=0))
            if magnifyBlue>=2:
                show[:,:,0]=np.maximum(show[:,:,0],np.roll(show[:,:,0],-1,axis=0))
            show[:,:,0]=np.maximum(show[:,:,0],np.roll(show[:,:,0],1,axis=1))
            if magnifyBlue>=2:
                show[:,:,0]=np.maximum(show[:,:,0],np.roll(show[:,:,0],-1,axis=1))
        if showrot:
            cv2.putText(show,'xangle %d'%(int(xangle/np.pi*180)),(30,showsz-30),0,0.5,cv2.cv.CV_RGB(255,0,0))
            cv2.putText(show,'yangle %d'%(int(yangle/np.pi*180)),(30,showsz-50),0,0.5,cv2.cv.CV_RGB(255,0,0))
            cv2.putText(show,'zoom %d%%'%(int(zoom*100)),(30,showsz-70),0,0.5,cv2.cv.CV_RGB(255,0,0))
    changed=True
    while True:
        if changed:
            render()
            changed=False
        cv2.imshow('show3d',show)
        if waittime==0:
            cmd=cv2.waitKey(10)%256
        else:
            cmd=cv2.waitKey(waittime)%256
        if cmd==ord('q'):
            break
        elif cmd==ord('Q'):
            sys.exit(0)

        if cmd==ord('t') or cmd == ord('p'):
            if cmd == ord('t'):
                if c_gt is None:
                    c0=np.zeros((len(xyz),),dtype='float32')+255
                    c1=np.zeros((len(xyz),),dtype='float32')+255
                    c2=np.zeros((len(xyz),),dtype='float32')+255
                else:
                    c0=c_gt[:,0]
                    c1=c_gt[:,1]
                    c2=c_gt[:,2]
            else:
                if c_pred is None:
                    c0=np.zeros((len(xyz),),dtype='float32')+255
                    c1=np.zeros((len(xyz),),dtype='float32')+255
                    c2=np.zeros((len(xyz),),dtype='float32')+255
                else:
                    c0=c_pred[:,0]
                    c1=c_pred[:,1]
                    c2=c_pred[:,2]
            if normalizecolor:
                c0/=(c0.max()+1e-14)/255.0
                c1/=(c1.max()+1e-14)/255.0
                c2/=(c2.max()+1e-14)/255.0
            c0=np.require(c0,'float32','C')
            c1=np.require(c1,'float32','C')
            c2=np.require(c2,'float32','C')
            changed = True



        if cmd==ord('n'):
            zoom*=1.1
            changed=True
        elif cmd==ord('m'):
            zoom/=1.1
            changed=True
        elif cmd==ord('r'):
            zoom=1.0
            changed=True
        elif cmd==ord('s'):
            cv2.imwrite('show3d.png',show)
        if waittime!=0:
            break
    return cmd
if __name__=='__main__':
    np.random.seed(100)
    showpoints(np.random.randn(2500,3))

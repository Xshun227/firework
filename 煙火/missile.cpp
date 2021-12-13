#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <new>
#include <cmath>
#include <graphics.h>
#include <time.h>
//#include <Windows.h>
using namespace std;
#define OutOfView 10000
#define SIZE 4 //scaling factor
const float Pi=3.1415926;
const float cAcc=0.3; //coefficient for the degree of acceleration function
const float lapse=0.05,tDelta=0.1; //data duration for missile data collecting �ɼu�ƾڦ������ƾګ���ɶ� 
const int wx=1400, wy=700; //�����j�p 
const int x_offset=20, y_offset=20;
int wx1=wx+2*x_offset;
int wy1=wy+2*y_offset;
int tSpan=10/tDelta; //max number of times to intercept the missile
int nCollect=tSpan/2; //number of missile data collected
int OFFSET=nCollect-20; //number of missile data ignored from beginning
int tDue=10/tDelta; //����s���̪��p�� (tDue+30) 
float fx=4000,fy=4000,fz=1000,lFocus=fx/3; //�J�I�εJ�Z 
int color;
int tCount=0;
int nSetup=30; //time needed to analyze data and set up for interceptor
float vMax=500; //initial value of velocity for interceptor
float aStart=Pi/6; //min elevated angle for interceptor
float dMin=20; //accepted distance while hitting between missile and interceptor
float dMax=500; //rejected distance while hitting between missile and interceptor
int i,n,ta,sc; //�Ϥ�����   sc= ring v�j�p 

class point
{
	public:
	float x,y,z;
	point()
	{
		x=0;
		y=0;
		z=0;
		return;
	}
		
	point(float xx,float yy,float zz)
	{
		x=xx;
		y=yy;
		z=zz;
		return;
	}	
	
	float length()
	{
		return pow(x*x+y*y+z*z,0.5);
	}
};

class vector
{
	public:
	point target;
	float length;
	
	vector(point p2)
	{
		target.x=p2.x;
		target.y=p2.y;
		target.z=p2.z;
		length=pow(pow(p2.x,2)+pow(p2.y,2)+pow(p2.z,2),0.5);
		return;
	}	
	
	vector(point p1,point p2)
	{

		target.x=p2.x-p1.x;
		target.y=p2.y-p1.y;
		target.z=p2.z-p1.z;
		length=pow(pow(target.x,2)+pow(target.y,2)+pow(target.z,2),0.5);
		return;	
	}
	
	vector()
	{
		target.x=0;
		target.y=0;
		target.z=0;
		length=0;
		return;
	}	
	
	vector uv()
	{
		point p;
		p.x=(target.x)/length;
		p.y=(target.y)/length;
		p.z=(target.z)/length;
		return p;
	}
	
	vector translate(point p)
	{
		p.x+=target.x;
		p.y+=target.y;
		p.z+=target.z;
		return p;
	}
	
	vector scale(float s)
	{
		point p;
		p.x=(target.x)*s;
		p.y=(target.y)*s;
		p.z=(target.z)*s;
		return p;
	}	
	
	vector rotate(int axis,float angle)//axis=1 for x, 2 for y, 3 for z axis
	{
		point p;
		switch(axis)
		{
		case 1: 
			p.x=target.x;
			p.y=target.y*cos(angle)-target.z*sin(angle);
			p.z=target.y*sin(angle)+target.z*cos(angle);
			break;
		case 2: 
			p.y=target.y;
			p.x=target.x*cos(angle)+target.z*sin(angle);
			p.z=-target.x*sin(angle)+target.z*cos(angle);
			break;
		case 3: 
			p.z=target.z;
			p.x=target.x*cos(angle)-target.y*sin(angle);
			p.y=target.x*sin(angle)+target.y*cos(angle);
			break;		
		}
		return p;
	}
	
	
	float inProduct(point p)
	{
		return (target.x)*(p.x)+(target.y)*(p.y)+(target.z)*(p.z);
	}
	
	vector outProduct(point p)
	{
		point q;
		q.x=(target.y)*(p.z)-(target.z)*(p.y);
		q.y=(target.z)*(p.x)-(target.x)*(p.z);
		q.z=(target.x)*(p.y)-(target.y)*(p.x);
		return q;
	}
	
	float angle(point p)
	{
		float a=inProduct(p)/length/p.length();
		return acos(a);
	}
};

class view
{
	public:
	float x,y;
	float xCenter,yCenter;
	point origin;//���������I3D�y�� 
	point vx,vy;//������ X �� Y �b���V�q 
	point fp; //focus point ��3D�y�� 
	float fl; //focus length
	
	view()
	{
		fp.x=fx;
		fp.y=fy;
		fp.z=fz;
		fl=lFocus;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp);
		origin=vTemp1.uv().scale(-fl).translate(fp).target;	
		point p0(0,0,-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.outProduct(vy).uv().target;
		return;
	}
	
	view(point p,point p1,float l)
	{
		fp.x=p.x;
		fp.y=p.y;
		fp.z=p.z;
		fl=l;
		xCenter=wx1/2;
		yCenter=wy1/2;
		vector vTemp1(fp,p1);
		origin=vTemp1.uv().scale(fl).translate(fp).target;	
		point p0(p1.x,p1.y,p1.z-1);
		vector vTemp2(pProj3D(p0));
		vector vTemp3(origin);
		vy=vTemp3.scale(-1).translate(vTemp2.target).uv().target;
		vx=vTemp1.scale(-1).outProduct(vy).uv().target;
		return;
	}
	
	point pProj3D(point p) //p�I��v�ᤧ3D�y�� 
	{
		vector vTemp0(fp);
		vector vTemp1(fp,p);
		point p0=vTemp1.uv().scale(fl/cos(vTemp1.uv().angle(vTemp0.scale(-1).uv().target))).translate(fp).target;
		return p0;//vTemp1.uv().scale(fl/cos(vTemp1.angle(vTemp0.scale(-1).target))).translate(fp).target;
	}
	
	void Proj2D(point p) // p�I��v�ᤧ�����y�� 
	{
		vector vT1(origin,p),vT2(fp,origin);
		if (vT1.inProduct(vT2.target) <= 0)
		{
			x=OutOfView;
			y=OutOfView;
			return;
		}
		point p0=pProj3D(p);
		vector vTemp1(origin,p0);
		x=xCenter+vTemp1.inProduct(vx);
		y=yCenter+vTemp1.inProduct(vy);
		return;	
	}
	
	void d3Point(point p) //�������e�X P�I 
	{
		point p0(0,0,0);
		vector v1(fp,p),v2(fp,p0);
		Proj2D(p);
		if (x==OutOfView) return;
		//putpixel(x,y,color);
		circle(x,y,2.5/(v1.length*cos(v1.angle(v2.target)))*fp.x*2+1); // �Ϥ����j�p(�b�|) �j��    ++++++++++++++++++++++++++++++
	
		return;
	}
	
	void d3Point1(point p) //�������e�X P�I 
	{
		point p0(0,0,0);
		vector v1(fp,p),v2(fp,p0);
		Proj2D(p);
		if (x==OutOfView) return;
		//putpixel(x,y,color);
		circle(x,y,0.2/(v1.length*cos(v1.angle(v2.target)))*fp.x*2+1); // �Ϥ����j�p(�b�|) �p��   +++++++++++++++++++++++++++++
		return;
	}

	
	void d3LineEmit(point p,point d) //p:source point  d:direction vector �������e�g�u 
	{
		Proj2D(p);
		if (x==OutOfView) return;
		float xT1=x,yT1=y;
		while (x>=0 && x<wx1 && y>=0 && y<wy1)
		{
			p.x+=d.x*10;
			p.y+=d.y*10;
			p.z+=d.z*10;
			Proj2D(p);	
			
			if (x==OutOfView) return;
			putpixel(x,y,10);
		}
		return;
	}
		
	void d3LineSeg(point p1,point p2) //�������e�u�q 
	{
		Proj2D(p1);
		if (x==OutOfView) return;
		float xT=x, yT=y;
		Proj2D(p2);
		if (x==OutOfView) return;
		float distance=pow(pow(x-xT,2)+pow(y-yT,2),0.5);
		if (x<(-distance) || xT<(-distance) || x>(wx1+distance) || xT>(wx1+distance)) return;
		if (y<(-distance) || yT<(-distance) || y>(wy1+distance) || yT>(wy1+distance)) return;
		line(x,y,xT,yT);
		return;
	}	
};


class firework //�����z�����Ϥ�  
{
	public:
	point *pCurrent;
	point *v;
	point *a;
	int *life;
	float g0,r0,r1,r2,n0;
	int *on;
	int nFire; //����Ӽ� 
	view w;
	
	firework(int n) //�]�w nFire �Ӥ��Ƥ���t��,��V �ΥͩR���� (�h�[����) 
	{
		int i;
		on=0;
		nFire=n;
		pCurrent=new point[n];
		v=new point[n];
		a=new point[n];
		life=new int[n];
		on=new int[n];
		g0=9.8;
		r0=0.04;
		r1=0.005;
		r2=0.0003;
		n0=-1;
		

		return;	
	}
 
	firework() //�]�w nFire �Ӥ��Ƥ���t��,��V �ΥͩR���� (�h�[����) 
	{
		int i;
		int n=1000+rand()%1000;
		on=0;
		nFire=n;
		pCurrent=new point[n];
		v=new point[n];
		a=new point[n];
		life=new int[n];
		on=new int[n];
		g0=9.8;  //���O 
		r0=0.04;  //�Ů���O 
		r1=0.005;
		r2=0.003;
		n0=-1-rand()%3;  //���O 
	
		return;	
	}
	
	void rcircle(int n) // �H����ηϤ�                  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	{
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
			vector vT(pT);
			v[i]=vT.uv().scale(rand()%30+70).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
	
	void circle(int n) // ��ηϤ�     
	{
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
			vector vT(pT);
			v[i]=vT.uv().scale(300).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
	void scircle(int n) // �p��ηϤ�     
	{
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
			vector vT(pT);
			v[i]=vT.uv().scale(50).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
	
	
	
	void allcircle(int n) // �j+��+�p��ηϤ�     
	{
		for (i=0;i<n;i++)
		{	
						
			if (i%3==2) //�j 
			{		
				point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
				vector vT(pT);
				v[i]=vT.uv().scale(200).target;
					life[i]=rand()%tDue+30;
				on[i]=0;
			}
			else if(i%3==1) //�� 
			{
				point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
				vector vT(pT);
				v[i]=vT.uv().scale(125).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}			
			else     //�p 
			{
				point pT(rand()%n-n/2,rand()%n-n/2,rand()%n-n/2);
				vector vT(pT);
				v[i]=vT.uv().scale(50).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}
			
		}
		
		return;	
	}
	
	void ring(int n) // �����Ϥ�      
	{
		
		for (i=0;i<n;i++)
		{
			point pT(rand()%n-n/2,rand()%n-n/2,0); //���� 
			vector vT(pT);
			v[i]=vT.uv().scale(rand()%50+50).target;
			life[i]=rand()%tDue+30;
			on[i]=0;
		}
		return;	
	}
		
	
	void pluse(int n) // �Q�r�Ϥ� 
	{
		
		for (i=0;i<n;i++)
		{	
						
			if (i%2==0)
			{		
				point pT(rand()%n-n/2,rand()%n-n/2,0); //���� 
				vector vT(pT);
				v[i]=vT.uv().scale(rand()%50+50).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}
			else 
			{
				point pT(0,0,rand()%n-n/2); //���� 
				vector vT(pT);
				v[i]=vT.uv().scale(rand()%100).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}
			
		}
		return;	
	}
		
	void allring(int n) // �j+��+�p�����Ϥ� 
	{
		
		for (i=0;i<n;i++)
		{	
			if(i%5==2) //�j 
			{
				point pT(rand()%n-n/2,rand()%n-n/2,0);
				vector vT(pT);
				v[i]=vT.uv().scale(100).rotate(1,11).rotate(3,2.5).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}
			else if(i%5==1) //�� 
			{
				point pT(rand()%n-n/2,rand()%n-n/2,0);
				vector vT(pT);
				v[i]=vT.uv().scale(50).rotate(1,11).rotate(3,2.5).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}			
			else     //�p 
			{
				point pT(rand()%n-n/2,rand()%n-n/2,0);
				vector vT(pT);
				v[i]=vT.uv().scale(10).rotate(1,11).rotate(3,2.5).target;
				life[i]=rand()%tDue+30;
				on[i]=0;
			}	
		}		
		return;	
	}
	void scring(int n) // �ۭq�����Ϥ� 
	{
		
		for (i=0;i<n;i++)
		{			
				point pT(rand()%n-n/2,rand()%n-n/2,0);
				vector vT(pT);
				v[i]=vT.uv().scale(sc).rotate(1,11).rotate(3,2.5).target;  //ratate 1=x 2=y 3=z
				life[i]=rand()%tDue+30;
				on[i]=0;
		} 
		return;	
	}
	
	
	void pInit(point pp) //�z���I 
	{
		int i;
		for (i=0;i<nFire;i++)
		{
			pCurrent[i]=pp;
			on[i]=1;
		}
		return;
	}
	
	void pNext(float t) //�p�� T�ɶ��ᤧ��m 
	{
		int i,n=7,g=4;
		float r;
		for (i=0;i<nFire;i++)
		{
			if (pCurrent[i].z>10 && life[i]>10)
				life[i]--;
			else 
			{
				on[i]=0;
				continue;
			}
			r = (pow(v[i].length(),2)*r2 + v[i].length()*r1 + r0)/v[i].length(); //�Ů���O 
			a[i].x=r*(-v[i].x)+n0*v[i].y/v[i].length()+(rand()%n-n/2); //���O  
			a[i].y=r*(-v[i].y)+n0*(-v[i].x)/v[i].length()+(rand()%n-n/2); //���O  
			a[i].z=r*(-v[i].z)-g0+(rand()%g-n/2);  //���O 
			pCurrent[i].x+=v[i].x*t+0.5*a[i].x*t*t;
			pCurrent[i].y+=v[i].y*t+0.5*a[i].y*t*t;
			pCurrent[i].z+=v[i].z*t+0.5*a[i].z*t*t;
			v[i].x=v[i].x+a[i].x*t;
			v[i].y=v[i].y+a[i].y*t;
			v[i].z=v[i].z+a[i].z*t;
		}
		//getch();
		return;		
	}	
	
	void show(int color) //��ܷϤ� 
	{
		int i;
		setcolor(color);
	
		for (i=0;i<nFire;i++)
		{
			if (on[i]==1)
				w.d3Point(pCurrent[i]);	
		}
		return;
	}

	void show1(int color) //��ܷϤ�(���y) 
	{
		int i;
		setcolor(color);
	
		for (i=0;i<nFire;i++)
		{
			if (on[i]==1)
			w.d3Point1(pCurrent[i]);	
		}
		return;
	}
};



class terrain
{
	public:
	terrain()
	{	
		srand (time(NULL));
		initwindow(wx1,wy1);
		setcolor(10);
		point p(fx,fy,fz),p1(0,0,0),d(1,0,0);
		float i;
		view w(p,p1,lFocus);
		//view w;
		p1.x=0;
		for (i=0;i<2*fy;i=i+200)
		{
			p1.y=i;
			w.d3LineEmit(p1,d);			
		}
		p1.y=0;
		d.x=0;
		d.y=1;
		for (i=0;i<2*fx;i=i+200)
		{
			p1.x=i;
			w.d3LineEmit(p1,d);			
		}
		p1.x=0;
		d.y=0;
		d.z=1;
		w.d3LineEmit(p1,d);	
		return;	
	}

	~terrain()
	{	
		getch();
		closegraph();
	}
 
};

class green_line  //�e�񩳽u 
{
	public:		
	green_line()
	{	
		srand (time(NULL));
		setcolor(10);
		point p(fx,fy,fz),p1(0,0,0),d(1,0,0);
		float i;
		view w(p,p1,lFocus);
		p1.x=0;
		for (i=0;i<2*fy;i=i+200)
		{
			p1.y=i;
			w.d3LineEmit(p1,d);			
		}
		p1.y=0;
		d.x=0;
		d.y=1;
		for (i=0;i<2*fx;i=i+200)
		{
			p1.x=i;
			w.d3LineEmit(p1,d);			
		}
		p1.x=0;
		d.y=0;
		d.z=1;
		w.d3LineEmit(p1,d);	
		return;	
	}

};

class back_ground  //�e�©� 
{
	public:	
	back_ground()
	{	
		srand (time(NULL));
		rectangle(0,0,2000,1000);
		setfillstyle(SOLID_FILL,0);
		floodfill(1000,500,1);		
		return;	
	}

};



void test1_firework()  //�Q�r�Ҧ� +++++++++++++++++++++++++++++++++++++++++++++++++++          test---1
{
	firework *f;              //tCount �Ϥ�(�e��)������ɶ�  tLimit �Ϥ������ɶ�  n ����Ӽ�  number �Ϥ�(�z���I)�Ӽ�  
	int z,m,i,j,tCount=2000,tLimit=tCount,n=rand()%50+150,number=15; //
	f=new firework[number];
	for (i=0;i<number;i++)
	{
		f[i].pluse(n); //�Ϥ����˦� 
		point p(2000-m,-1000+m,500+z); //�Ϥ��H������m 
		f[i].pInit(p); //�z���I 
		m+=350;
		if(z<1000 && m<1500)
		{
			z+=200;	
		}
		else
		{
			z-=200;
		}
	}
  	for (j=0;j<number;j+=1)    // j=�Ϥ��z���I�Ӽ� 
  	{
  		tLimit-=rand()%40+20; //�Ʀr�V�p �t�׶V�� 
  		cout<<tLimit<<","<<j<<","<<number<<","<<tCount<<endl;
    	while (tCount>tLimit && tCount>=0)	
    	{			
			for (i=0;i<j;i++)
			f[i].show(i%6+9);    //�Ϥ����C�� 
				
			delay(20*lapse);
			
			for (i=0;i<j;i++)			
			f[i].show(0);        //�\���Ϥ� 
			if(ta==1)
			{
			for (i=0;i<j;i++)			
			f[i].show1(i%6+9);   //���ͧ��y  	
			}								
			for (i=0;i<j;i++)			
			f[i].pNext(tDelta); //���ʨ�t��᪺��m 
			
			tCount--;
		}
  	}	
	return;
};

void test2_firework()  // �H������ +++++++++++++++++++++++++++++++++++++++++++++++++++          test---2 
{
	firework *f;
	int k=0,z,m,i,j,tCount=2000,tLimit=tCount,n=rand()%50+350,number=15; //tCount �Ϥ�(�e��)������ɶ�  tLimit �Ϥ������ɶ� 
	f=new firework[number];
	for (i=0;i<number;i++)
	{

		f[i].rcircle(n);
		point p(rand()%4000-2000,rand()%4000-2000,rand()%600+500);
		f[i].pInit(p); //�z���I 
		
			
	}
  	for (j=0;j<number;j+=rand()%1+1)
  	{
  		tLimit-=rand()%40+40;  // tLimit �Ϥ������ɶ� 
  		cout<<tLimit<<","<<j<<","<<number<<","<<tCount<<endl;
    	while (tCount>tLimit && tCount>=0)	
    	{			
			for (i=0;i<j;i++)
			f[i].show(i%76+9);    //�Ϥ����C�� 
				
			delay(20*lapse);
		
			for (i=0;i<j;i++)			
			f[i].show(0);        //�\���Ϥ� 
			if(ta==1)
			{
			for (i=0;i<j;i++)			
			f[i].show1(i%6+9);   //���ͧ��y  	
			}							
			for (i=0;i<j;i++)			
			f[i].pNext(tDelta); //���ʨ�t��᪺��m 
		
			tCount--;
		}
  	}	
		return;
};

void test3_firework() //���l�Ҧ� +++++++++++++++++++++++++++++++++++++++++++++++++++          test---3 
{
	firework *f;
	int k=0,m=0,z=0,i,j,tCount=2000,tLimit=tCount,n=rand()%50+100,number=30; //tCount �Ϥ�(�e��)������ɶ�  tLimit �Ϥ������ɶ� 
	f=new firework[number];
	for (i=0;i<number;i++)
	{
		k++;
    	if(k%2==0)
    	{ 
    		f[i].circle(n);
			point p(2000-m,-500+m,400+z);
			f[i].pInit(p); //�z���I 
			m+=400;
			z+=100;
    		if(z>1000)
			{
				m+=700;
				z-=1000;
			}
		} 
    	else
		{ 
			f[i].scircle(n);
			point p(2000-m,-500+m,400+z);
			f[i].pInit(p); 

		}
	}

  	for (j=0;j<number;j+=2)
  	{
  		tLimit-=40; 
  		cout<<tLimit<<","<<j<<","<<number<<","<<tCount<<endl;
    	while (tCount>tLimit && tCount>=0)	
    	{			
			for (i=0;i<j;i++)
			f[i].show(i%6+9);    //�Ϥ����C�� 
				
			delay(20*lapse);
		
			for (i=0;i<j;i++)			
			f[i].show(0);        //�\���Ϥ� 
			if(ta==1)
			{
			for (i=0;i<j;i++)			
			f[i].show1(i%6+9);   //���ͧ��y  	
			}				
			for (i=0;i<j;i++)			
			f[i].pNext(tDelta); //���ʨ�t��᪺��m 
		
			tCount--;
		}
  	}	
		return;
};

void test4_firework()  //�H���h���� +++++++++++++++++++++++++++++++++++++++++++++++++++          test---4 
{
    firework *f;
    int d=0,c=0,k=0,i,j,tCount=1000,tLimit=tCount,n=rand()%100+300,number=20; //tCount �Ϥ�(�e��)������ɶ� n=rand()%100+100
    f=new firework[number];
    for (i=0;i<number;i++)
    {   
    
    	f[i].allring(n);
        point p(rand()%4000-2000,rand()%4000-2000,rand()%600+500);
    	f[i].pInit(p); //�z���I 

    }
  	for (j=0;j<number;j+=2)
  	{
  		tLimit-=30; 
  		cout<<tLimit<<","<<j<<","<<number<<","<<tCount<<endl;
    	while (tCount>tLimit && tCount>=0)	
    	{			
			for (i=0;i<j;i++)
			f[i].show(i%6+10);    //�Ϥ����C�� 
				
			delay(20*lapse);
			
			for (i=0;i<j;i++)			
			f[i].show(0);        //�\���Ϥ� 
			if(ta==1)
			{
			for (i=0;i<j;i++)			
			f[i].show1(i%6+9);   //���ͧ��y  	
			}								
			for (i=0;i<j;i++)			
			f[i].pNext(tDelta); //���ʨ�t��᪺��m 
			
			tCount--;
		}
  	}	
	return;
};

void test5_firework()  //�m��h���� +++++++++++++++++++++++++++++++++++++++++++++++++++          test---5  
{
    firework *f;
    int d=0,c=0,k=0,i,j,m,tCount=2000,tLimit=tCount,n=200,number=25; //tCount �Ϥ�(�e��)������ɶ� n=rand()%100+100
    f=new firework[number];
    for (i=0;i<number;i++)
    {   
		if(i%5==4)
    	{ 
    		sc=500;
    		f[i].scring(n);
			point p(1500-m,500+m,800);
			f[i].pInit(p); //�z���I 
			m+=1200;
			
		} 
		else if(i%5==3)
		{ 
			sc=400;
			f[i].scring(n);
			point p(1500-m,500+m,800);
			f[i].pInit(p); 

		}
		else if(i%5==2)
		{ 
			sc=100;
			f[i].scring(n);
			point p(1500-m,500+m,800);
			f[i].pInit(p); 

		}
		else if(i%5==1)
		{ 
			sc=50;
			f[i].scring(n);
			point p(1500-m,500+m,800);
			f[i].pInit(p); 

		}
    	else
		{ 
			sc=5;
			f[i].scring(50);
			point p(1500-m,500+m,800);
			f[i].pInit(p); 

		}


    }
  	for (j=0;j<number;j+=5)
  	{
  		tLimit-=10; 
  		cout<<tLimit<<","<<j<<","<<number<<","<<tCount<<endl;
    	while (tCount>tLimit && tCount>=0)	
    	{			
			for (i=0;i<j;i++)
			f[i].show(i%5+10);    //�Ϥ����C�� 
				
			delay(20*lapse);
			
			for (i=0;i<j;i++)			
			f[i].show(0);        //�\���Ϥ� 
			
			if(ta==1)
			{
			for (i=0;i<j;i++)			
			f[i].show1(i%6+9);   //���ͧ��y  	
			}	
						
			for (i=0;i<j;i++)			
			f[i].pNext(tDelta); //���ʨ�t��᪺��m 
			
			tCount--;
		}
  	}	
	return;
};

int main()
{	
    int c; 
	FILE *fp;
	fp=fopen("firework123.txt","r");
	fscanf(fp,"%d",&c);
	fscanf(fp,"%d",&ta);
	printf("%d\n",c);
	terrain t;
	if(c==1) //�Q�r�Ҧ� 
    { 
		test1_firework();
    } 

	if(c==2) //�H���Ҧ� 
    { 
		test2_firework();
	} 
	if(c==3) //���l�Ҧ� 
    { 
		test3_firework();
	} 
	if(c==4) //�j+��+�p��μҦ� 
    { 
		test4_firework();
	} 
	if(c==5) //�j+��+�p�����Ҧ� 
    { 
		test5_firework();
	} 
	if(c==6) //���Ҧ� 
    { 
    	test1_firework();
		back_ground a;
		green_line e;
		test2_firework();
		back_ground b;
		green_line f;
		test3_firework();
		back_ground c;
		green_line g;
		test4_firework();
		back_ground d;
		green_line h;  
		test5_firework(); 
	} 
	if(c==7) //���ƥ��Ҧ� 
	{
	    while(1)
    	{
    	test1_firework();
		back_ground a;
		green_line e;
		test2_firework();
		back_ground b;
		green_line f;
		test3_firework();
		back_ground c;
		green_line g;
		test4_firework();
		back_ground d;
		green_line h;  
		test5_firework(); 

		}
	}

	fclose(fp);

	return 1;
}

/* ���O��
	6	(c)  case����  1~7 
	0	(ta) ���y   1(�}��) 0(����) 
*/ 


https://ln5.sync.com/dl/17f69ba50/87wpj6pu-75xwikr6-8dum8653-4wt8eemr
# ByteTrack with (COCO) labels

![output image]( https://qengineering.eu/github/BYTEtrackGraph.jpg )

## ByteTrack expanded with COCO or VOC labels. <br/>
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)<br/><br/>

------------

## Introduction.
ByteTrack is one of the best tracker in the field. However, it has two shortcomings.

#### Thread safety.
The C++ code uses function calls to static class routines.<br>
It works well if you have one tracker running. With multiple instances of the trackers, you might face unpredictable situations.<br>
Because every tracker, every thread, shares the same static variables, it can lead to rare conditions.<br>
In this C++ implementation, we have removed all these potentially harmful calls.<br>

#### Lack of labels.
ByteTrack only works with the bounding boxes of the objects and their probability. It takes no notion of the object class.<br>
In most cases, it is no problem. Often, you have only one class in which you are interested, like pedestrians or football players.<br>
However, on some occasions, the bounding boxes of different objects might coincide, as can be seen in the animation below.<br><br>
![](https://qengineering.eu/github/InTrack.gif)<br>
A person is being marked as a car just because, at some point, their bounding boxes are (almost) identical.<br><br>

We add the object class `obj_id` as an additional variable to the STrack class.<br>
```cpp
//---------------------------------------------------------------------------
class STrack
{
public:
	STrack(Ttlwh tlwh_, float score, int obj_id, BYTETracker* TrackInstance);
	~STrack();
......
public:
......
	int obj_id;

	Ttlwh _tlwh;
	Ttlwh tlwh;
	Ttlbr tlbr;
......
private:
    BYTETracker* myTrackInstance;               // Reference to BYTETracker instance
	byte_kalman::KalmanFilter kalman_filter;
};
//---------------------------------------------------------------------------
```
During Update(), this integer is filled with the object label.<br>
It doesn't matter which kind of set you use. COCO, VOC or another.<br>
```ccp
//---------------------------------------------------------------------------
void BYTETracker::update(vector<bbox_t>& objects)
{

	////////////////// Step 1: Get detections //////////////////
	this->frame_id++;
......
	if (objects.size() > 0)
	{
		for(size_t i = 0; i < objects.size(); i++){
			Ttlwh a;
			a.t=objects[i].x;
			a.l=objects[i].y;
			a.w=objects[i].w;
			a.h=objects[i].h;

			float score = objects[i].prob;
			int obj_id  = objects[i].obj_id;

			STrack strack(a, score, obj_id, this);
			if (score >= track_thresh) detections.push_back(strack);
			else  			   detections_low.push_back(strack);
		}
	}
```
The distinction between object classes is done by giving a penalty in the IoU calculus when classes don't match.<br>
```cpp
for (size_t k = 0; k < btlbrs.size(); k++){
    vector<float> ious_tmp;
    float box_area = (btlbrs[k].b - btlbrs[k].t + 1)*(btlbrs[k].r - btlbrs[k].l + 1);
    for (size_t n = 0; n < atlbrs.size(); n++){
        float iw = min(atlbrs[n].b, btlbrs[k].b) - max(atlbrs[n].t, btlbrs[k].t) + 1;
        if (iw > 0){
            float ih = min(atlbrs[n].r, btlbrs[k].r) - max(atlbrs[n].l, btlbrs[k].l) + 1;
            if(ih > 0){
                float ua = (atlbrs[n].b - atlbrs[n].t + 1)*(atlbrs[n].r - atlbrs[n].l + 1) + box_area - iw * ih;
                ious[n][k] = iw * ih / ua;

                if(atlbrs[n].c!=btlbrs[k].c){    //not same object class? -> penalty
                    ious[n][k] *=0.5;
                }
            }
            else ious[n][k] = 0.0;
        }
        else ious[n][k] = 0.0;
    }
}
```

![](https://qengineering.eu/github/OutTrack.gif)<br>


#### Final remarks.

The original code uses `std::vector<float>` to hold the bounding boxes.<br>
We use a simple `struct` because of the fixed number of elements.<br>
A `std::vector` requires more overhead. And the code becomes more readable.<br><br>
We only adapted the C++ implementation. Python isn't supported. If someone likes to port the code to Python, be our guest.<br>

```
BYTETrack
├── include
│   ├── BYTETracker.h
│   ├── dataType.h
│   ├── kalmanFilter.h
│   ├── lapjv.h
│   └── STrack.h
├── LICENSE
├── README.md
└── src
    ├── BYTETracker.cpp
    ├── kalmanFilter.cpp
    ├── lapjv.cpp
    ├── STrack.cpp
    └── utils.cpp
```


------------

[![paypal](https://qengineering.eu/images/TipJarSmall4.png)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=CPZTM5BB3FCYL) 



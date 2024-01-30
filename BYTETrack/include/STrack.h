#pragma once

#include <opencv2/opencv.hpp>
#include "kalmanFilter.h"
#include "dataType.h"

using namespace cv;
using namespace std;

class BYTETracker;

//---------------------------------------------------------------------------
enum TrackState { New = 0, Tracked, Lost, Removed };
//---------------------------------------------------------------------------
class STrack
{
public:
	STrack(Ttlwh tlwh_, float score, int obj_id, BYTETracker* TrackInstance);
	~STrack();
	void static_tlwh();
	void static_tlbr();
	void mark_lost();
	void mark_removed();
	int end_frame();

	void activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id);
	void re_activate(STrack &new_track, int _frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);

public:
	bool is_activated;
	int track_id;
	int state;
	int obj_id;

	Ttlwh _tlwh;
	Ttlwh tlwh;
	Ttlbr tlbr;

	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;

private:
    BYTETracker* myTrackInstance;               // Reference to BYTETracker instance
	byte_kalman::KalmanFilter kalman_filter;
};
//---------------------------------------------------------------------------

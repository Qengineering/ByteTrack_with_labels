#include "STrack.h"
#include "BYTETracker.h"
//---------------------------------------------------------------------------
// STrack
//---------------------------------------------------------------------------
STrack::STrack(Ttlwh tlwh_, float score, int obj_id, BYTETracker* TrackInstance) : myTrackInstance(TrackInstance)
{
	_tlwh=tlwh_;

	is_activated = false;
	track_id = 0;
	state = TrackState::New;

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	this->obj_id = obj_id;
	start_frame = 0;
}
//---------------------------------------------------------------------------
STrack::~STrack()
{
}
//---------------------------------------------------------------------------
void STrack::activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = myTrackInstance->ID_count++;

	vector<float> xyah = tlwh_to_xyah(this->_tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}
//---------------------------------------------------------------------------
void STrack::re_activate(STrack &new_track, int _frame_id, bool new_id)
{
	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state        = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id     = _frame_id;
	this->score        = new_track.score;

	if (new_id) this->track_id = myTrackInstance->ID_count++;
}
//---------------------------------------------------------------------------
void STrack::update(STrack &new_track, int _frame_id)
{
	this->frame_id = _frame_id;
	this->tracklet_len++;

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}
//---------------------------------------------------------------------------
void STrack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh = _tlwh;
		return;
	}

	tlwh.t = mean[0];
	tlwh.l = mean[1];
	tlwh.w = mean[2];
	tlwh.h = mean[3];

	tlwh.w *= tlwh.h;
	tlwh.t -= tlwh.w / 2;
	tlwh.l -= tlwh.h / 2;
}
//---------------------------------------------------------------------------
void STrack::static_tlbr()
{
    tlbr.t=tlwh.t;
    tlbr.l=tlwh.l;
    tlbr.b=tlwh.t+tlwh.w;
    tlbr.r=tlwh.l+tlwh.h;
    tlbr.c=this->obj_id;
}
//---------------------------------------------------------------------------
void STrack::mark_lost()
{
	state = TrackState::Lost;
}
//---------------------------------------------------------------------------
void STrack::mark_removed()
{
	state = TrackState::Removed;
}
//---------------------------------------------------------------------------
int STrack::end_frame()
{
	return this->frame_id;
}
//---------------------------------------------------------------------------

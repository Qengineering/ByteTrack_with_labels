#include "BYTETracker.h"
#include "STrack.h"
#include <plog/Log.h>
#include <fstream>
//---------------------------------------------------------------------------
// BYTETracker
//---------------------------------------------------------------------------
BYTETracker::BYTETracker(void)
{
	ID_count = 0;
}
//---------------------------------------------------------------------------
BYTETracker::~BYTETracker()
{
}
//---------------------------------------------------------------------------
void BYTETracker::Init(int frame_rate, int track_buffer)
{
	track_thresh = 0.1;
	high_thresh = 0.5;
	match_thresh = 0.8;

	frame_id = 0;
	max_time_lost = int(frame_rate / 30.0 * track_buffer);

    PLOG_INFO << "Init ByteTrack (max time lost: "<<max_time_lost<<")";
}
//---------------------------------------------------------------------------
void BYTETracker::update(vector<bbox_t>& objects)
{
    float fiou, miou;
    int m;
    Ttlwh a;

	////////////////// Step 1: Get detections //////////////////
	this->frame_id++;
	vector<STrack> activated_stracks;
	vector<STrack> refind_stracks;
	vector<STrack> removed_stracks;
	vector<STrack> lost_stracks;
	vector<STrack> detections;
	vector<STrack> detections_low;

	vector<STrack> detections_cp;
	vector<STrack> tracked_stracks_swap;
	vector<STrack> resa, resb;
	vector<STrack> output_stracks;

	vector<STrack*> unconfirmed;
	vector<STrack*> tracked_stracks;
	vector<STrack*> strack_pool;
	vector<STrack*> r_tracked_stracks;

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
			else  			           detections_low.push_back(strack);
		}
	}

	// Add newly detected tracklets to tracked_stracks
	for(size_t i = 0; i < this->tracked_stracks.size(); i++){
		if (!this->tracked_stracks[i].is_activated)
			unconfirmed.push_back(&this->tracked_stracks[i]);
		else
			tracked_stracks.push_back(&this->tracked_stracks[i]);
	}

	////////////////// Step 2: First association, with IoU //////////////////
	strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);

//	multi_predict(strack_pool, this->kalman_filter);
	for(size_t i = 0; i < strack_pool.size(); i++){
		if(strack_pool[i]->state != TrackState::Tracked) strack_pool[i]->mean[7] = 0;
		this->kalman_filter.predict(strack_pool[i]->mean, strack_pool[i]->covariance);
		strack_pool[i]->static_tlwh();
		strack_pool[i]->static_tlbr();
	}

	vector<vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

	vector<vector<int> > matches;
	vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

	for (size_t i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	////////////////// Step 3: Second association, using low score dets //////////////////
	for(size_t i = 0; i < u_detection.size(); i++){
		detections_cp.push_back(detections[u_detection[i]]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());

	for(size_t i = 0; i < u_track.size(); i++){
		if (strack_pool[u_track[i]]->state == TrackState::Tracked){
			r_tracked_stracks.push_back(strack_pool[u_track[i]]);
		}
	}

	dists.clear();
	dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for(size_t i = 0; i < matches.size(); i++){
		STrack *track = r_tracked_stracks[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	for(size_t i = 0; i < u_track.size(); i++){
		STrack *track = r_tracked_stracks[u_track[i]];
		if (track->state != TrackState::Lost)
		{
			track->mark_lost();
			lost_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	vector<int> u_unconfirmed;
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

	for(size_t i = 0; i < matches.size(); i++){
		unconfirmed[matches[i][0]]->update(detections[matches[i][1]], this->frame_id);
		activated_stracks.push_back(*unconfirmed[matches[i][0]]);
	}

	for(size_t i = 0; i < u_unconfirmed.size(); i++){
		STrack *track = unconfirmed[u_unconfirmed[i]];
		track->mark_removed();
		removed_stracks.push_back(*track);
	}

	////////////////// Step 4: Init new stracks //////////////////
	for(size_t i = 0; i < u_detection.size(); i++){
		STrack *track = &detections[u_detection[i]];
		if (track->score < this->high_thresh)
			continue;
		track->activate(this->kalman_filter, this->frame_id);
		activated_stracks.push_back(*track);
	}

	////////////////// Step 5: Update state //////////////////
	for(size_t i = 0; i < this->lost_stracks.size(); i++){
		if (this->frame_id - this->lost_stracks[i].end_frame() > this->max_time_lost)
		{
			this->lost_stracks[i].mark_removed();
			removed_stracks.push_back(this->lost_stracks[i]);
		}
	}

	for(size_t i = 0; i < this->tracked_stracks.size(); i++){
		if (this->tracked_stracks[i].state == TrackState::Tracked){
			tracked_stracks_swap.push_back(this->tracked_stracks[i]);
		}
	}
	this->tracked_stracks.clear();
	this->tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

	this->tracked_stracks = joint_stracks(this->tracked_stracks, activated_stracks);
	this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

	this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
	for(size_t i = 0; i < lost_stracks.size(); i++){
		this->lost_stracks.push_back(lost_stracks[i]);
	}

	this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);
	for(size_t i = 0; i < removed_stracks.size(); i++){
		this->removed_stracks.push_back(removed_stracks[i]);
	}

	remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);

	this->tracked_stracks.clear();
	this->tracked_stracks.assign(resa.begin(), resa.end());
	this->lost_stracks.clear();
	this->lost_stracks.assign(resb.begin(), resb.end());

	for(size_t i = 0; i < this->tracked_stracks.size(); i++){
		if (this->tracked_stracks[i].is_activated){
			output_stracks.push_back(this->tracked_stracks[i]);
		}
	}

    //repair labels
    //re-use state to hold allocated tracks
    for(size_t i=0; i<output_stracks.size(); i++) output_stracks[i].state=-1;

    for(size_t n=0; n<objects.size();  n++){
        a.t=objects[n].x;   a.l=objects[n].y;
        a.w=objects[n].w;   a.h=objects[n].h;
        miou=0.0; m=-1;
        for(size_t i=0; i<output_stracks.size(); i++) {
            fiou = IoU(output_stracks[i].tlwh, a);
            if(objects[n].obj_id == (unsigned int)output_stracks[i].obj_id){
                if(fiou>miou){ miou=fiou; m=(int)i; }
            }
        }
        if(miou > 0.1){
            //found in output stacks, so set
            output_stracks[m].state=m;
        }
        else{
            //not found, look for best bbox match
            //can also be an object leaving the scene.
            for(size_t i=0; i<output_stracks.size(); i++) {
                if(output_stracks[i].state<0){
                    fiou = IoU(output_stracks[i].tlwh, a);
                    if(fiou>miou){ miou=fiou; m=(int)i; }
                }
            }
            if(miou > 0.5){
                //found replacement
                output_stracks[m].state  = m;
                output_stracks[m].obj_id = objects[n].obj_id;
                //repair the other Straks
                //vector<STrack> tracked_stracks;
                for(size_t z=0; z<this->tracked_stracks.size(); z++) {
                    if(this->tracked_stracks[z].track_id == output_stracks[m].track_id){
                        this->tracked_stracks[z].obj_id = output_stracks[m].obj_id;
                    }
                }
                //vector<STrack> lost_stracks;
                for(size_t z=0; z<this->lost_stracks.size(); z++) {
                    if(this->lost_stracks[z].track_id == output_stracks[m].track_id){
                        this->lost_stracks[z].obj_id = output_stracks[m].obj_id;
                    }
                }
                //vector<STrack> removed_stracks;
                for(size_t z=0; z<this->removed_stracks.size(); z++) {
                    if(this->removed_stracks[z].track_id == output_stracks[m].track_id){
                        this->removed_stracks[z].obj_id = output_stracks[m].obj_id;
                    }
                }
            }
        }
    }

	objects.clear();

    for(size_t i = 0; i < output_stracks.size(); i++) {
        bbox_t B;
        B.x       =output_stracks[i].tlwh.t;
        B.y       =output_stracks[i].tlwh.l;
        B.w       =output_stracks[i].tlwh.w;
        B.h       =output_stracks[i].tlwh.h;
        B.obj_id  =output_stracks[i].obj_id;
        B.prob    =output_stracks[i].score;
        B.track_id=output_stracks[i].track_id;
        objects.push_back(B);
    }
}
//---------------------------------------------------------------------------

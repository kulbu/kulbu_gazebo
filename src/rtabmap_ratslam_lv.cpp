// RTABMap signature, hypothesis to Ratslam LocalView ID.

//#include <iterator>
//#include <vector>
//#include <unistd.h>

#include <ros/ros.h>
//#include <rtabmap/core/Statistics.h>
#include <rtabmap_ros/Info.h>
#include <rtabmap_ros/AllowLoop.h>
//#include <rtabmap_ros/RejectLoop.h>
#include <ratslam_ros/ViewTemplate.h>
#include <ratslam_ros/GetDistance.h>

#include <kulbu_gazebo/DistanceMatrix.h>


/*
// Interprocess communication accessible to RTABMap closure checking.
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

//using namespace boost::interprocess;
//Alias an STL compatible allocator of ints that allocates ints from the managed
//shared memory segment.  This allocator will allow to place containers
//in managed shared memory segments
typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;

//Alias a vector that uses the previous STL-like allocator
typedef boost::interprocess::vector<int, ShmemAllocator> ShmenVector;

boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "ratmap_ratslam_lv", 4096);

//Initialize shared memory STL-compatible allocator
const ShmemAllocator alloc_inst (segment.get_segment_manager());

//Construct a shared memory
ShmenVector *frames = 
    segment.construct<ShmenVector>("frames") //object name
                               (alloc_inst);//first ctor parameter
*/
std::vector<int> frames;

int queue_size;
double loop_thr;
int distance_thr;

bool allow_loops = true;

int last_id = 0;
int last_high_id = 0;
int frame_id = 0;
int matched_id = 0;

int curr_id = 0;
int highest_id = 0;
double highest_val = 0.0;

int allow_old = 0;
int allow_new = 0;

int wait_cnt = 0;

/**
 * Retrieve local RatSLAM friendly representation of RTABMap signature_id.
 * @method getLocalFrame
 * @param {int} rtabmap_id
 * @return {int}
 */
int getLocalFrame(int rtabmap_id)
{
    int res = 0;
    std::vector<int>::iterator it = std::find(frames.begin(), frames.end(), (int)rtabmap_id);
    //boost::interprocess::vector<int, ShmemAllocator>::iterator it = std::find(frames->begin(), frames->end(), (int)rtabmap_id);
    if ((int)it[0] == (int)rtabmap_id) {
        res = it - frames.begin();
        ROS_DEBUG_STREAM( "getLocalFrame: res=" << res << " val=" << it[0] );
    }
    return res;
}

/**
 * Extract highest hypothesis, it's value and the current reference id from /rtabmap/info
 * @method InfoCallback
 * @param {rtabmap_ros::Info} info
 */
void InfoCallback(const rtabmap_ros::InfoConstPtr& info) {
    // First, remember so we can query distances later.
    curr_id = info->refId;

    // Save new signature id and pass on local sequential representation.
    if (last_id < curr_id) {
        //frames->resize(frames->size() + 1);
        //frames->push_back((int)info->refId);
        //frame_id = frames->size();
        frames.push_back((int)curr_id);
        frame_id = frames.size();
        ROS_DEBUG_STREAM( "RT:RAT:New frame curr_id=" << curr_id << " last_id=" << last_id << " frame_id=" << frame_id );
    }

    // Look up signature id's local version and link them in RatSLAM.
    highest_id = info->statsValues[6];
    highest_val = info->statsValues[7];
    // Not same as last and over threshold.
    if (highest_id != last_high_id && highest_val > loop_thr) {
        matched_id = getLocalFrame(highest_id);
    }




// DEBUG: Allow all..
//allow_old = highest_id;
//allow_new = info->refId;



    // Finally, remember so we know what is new.
    last_id = info->refId;
    ROS_DEBUG_STREAM( "RT:RAT: frame_id=" << frame_id << " matched_id=" << matched_id << " highest_id=" << highest_id << " highest_val=" << highest_val );

}

int main( int argc, char** argv) {

    /*
    //Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() { boost::interprocess::shared_memory_object::remove("ratmap_ratslam_lv"); }
        ~shm_remove(){ boost::interprocess::shared_memory_object::remove("ratmap_ratslam_lv"); }
    } remover;
    */

    // Ros pub/sub.
    if (!ros::isInitialized()) {
        ros::init(argc, argv, "rtabmap_ratslam_lv");
    }

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // FIXME: not working...
    pn.param("queue_size", queue_size, 4);
    pn.param("LoopThr", loop_thr, 0.11);
    pn.param("DistanceThr", distance_thr, 2000);
/*
    std::string key;
    if (n.searchParam("queue_size", key))
    {
        ROS_INFO_STREAM( "RT:RAT: Parameters key=" << key);
        //std::string val;
        n.getParam(key, queue_size);
    }
*/
    
    ROS_INFO_STREAM( "RT:RAT: Parameters queue_size=" << queue_size << " loop_thr=" << loop_thr << " distance_thr=" << distance_thr );

    ros::Publisher pub_dm = n.advertise<kulbu_gazebo::DistanceMatrix>("/rtab_rat/DistanceMatrix", queue_size);
    
    ros::Publisher pub_lv = n.advertise<ratslam_ros::ViewTemplate>("/ratslam/LocalView/Template", queue_size);
    //(int)queue_size/2
    ros::Subscriber sub_info = n.subscribe("/rtabmap/info", 1, InfoCallback);

    ros::ServiceClient client_allow = n.serviceClient<rtabmap_ros::AllowLoop>("/rtabmap/allow_loop");
    ros::ServiceClient client_distance = n.serviceClient<ratslam_ros::GetDistance>("/ratslam/ExperienceMap/GetDistance");

    ros::Time start_time, current_time, last_time;
    start_time = ros::Time::now();
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Main loop
    //ros::spin();
    ros::Rate r(20.0);
    while (n.ok()) {

        current_time = ros::Time::now();

        // Determine if a potential loop closure is too far away, before adding the new frame.
        if (matched_id > 0) {
            // `loopClosure` will never be set as we're rejecting by default in RTABMap via a whitelist.
            // Better implementation would be RatSLAM integrated into RTABMap so hypothesis can be rejected in real-time.
            if (curr_id > 0) {
                // Call `/ratslam/ExperienceMap/GetDistance` to gather temporal distances.

                int a = 0;

                // 1 +/-
                int b = 0;
                int c = 0;
                int d = 0;
                int e = 0;

                // 2 +/-
                int f = 0;
                int g = 0;
                int h = 0;
                int i = 0;

                ratslam_ros::GetDistance srv;

                srv.request.id1 = curr_id;
                srv.request.id2 = highest_id;
                if (client_distance.call(srv))
                {
                    a = srv.response.distance;
                }

                // Last highest hypothesis distance vrs this highest distance.
                srv.request.id1 = curr_id;
                srv.request.id2 = last_high_id;
                if (client_distance.call(srv))
                {
                    b = srv.response.distance;
                }

/*
                // Distance to last
                srv.request.id1 = curr_id;
                srv.request.id2 = curr_id-1;
                if (client_distance.call(srv))
                {
                    b = srv.response.distance;
                }

                srv.request.id1 = curr_id;
                srv.request.id2 = highest_id-1;
                if (client_distance.call(srv))
                {
                    b = srv.response.distance;
                }

                srv.request.id1 = curr_id;
                srv.request.id2 = highest_id-2;
                if (client_distance.call(srv))
                {
                    b = srv.response.distance;
                }

*/


/*
                // 1 +/-
                srv.request.id1 = curr_id+1;
                srv.request.id2 = highest_id+1;
                if (client_distance.call(srv))
                {
                    b = srv.response.distance;
                }
                srv.request.id1 = curr_id-1;
                srv.request.id2 = highest_id-1;
                if (client_distance.call(srv))
                {
                    c = srv.response.distance;
                }

                srv.request.id1 = curr_id-1;
                srv.request.id2 = highest_id+1;
                if (client_distance.call(srv))
                {
                    d = srv.response.distance;
                }
                srv.request.id1 = curr_id+1;
                srv.request.id2 = highest_id-1;
                if (client_distance.call(srv))
                {
                    e = srv.response.distance;
                }

                // 2 +/-
                srv.request.id1 = curr_id+2;
                srv.request.id2 = highest_id+2;
                if (client_distance.call(srv))
                {
                    f = srv.response.distance;
                }
                srv.request.id1 = curr_id-2;
                srv.request.id2 = highest_id-2;
                if (client_distance.call(srv))
                {
                    g = srv.response.distance;
                }

                srv.request.id1 = curr_id-2;
                srv.request.id2 = highest_id+2;
                if (client_distance.call(srv))
                {
                    h = srv.response.distance;
                }
                srv.request.id1 = curr_id+2;
                srv.request.id2 = highest_id-2;
                if (client_distance.call(srv))
                {
                    i = srv.response.distance;
                }
*/



                kulbu_gazebo::DistanceMatrix dm;
                //dm.header.stamp = current_time;
                //dm.header.frame_id = "";
                dm.a = a;
                dm.b = b;
                dm.c = c;
                dm.d = d;
                dm.e = e;
                dm.f = f;
                dm.g = g;
                dm.h = h;
                dm.i = i;

                if (b != 0) dm.calc1 = ((float)a/(float)b)*100;
                // TODO: cumulative up and down of calc1.

                //if (b != 0) dm.calc2 *= ((float)a/(float)b);
                
                //dm.h_i = a * 1/(float)dm.b;

// TODO: rate of change, this `h_i` vrs last
                //if (dm.b != 0) dm.d_e = dm.h_i / 
                 //   last dm.h_i;

                //if (e != 0) dm.d_e = (float)d/(float)e;
                //if (i != 0) dm.h_i = (float)h/(float)i;
                /*
                ros::Duration diff = current_time - start_time;
                if (diff.toSec() > 0) {
                    dm.d_e = (double)a/(diff.toSec()-40000);
                    ROS_DEBUG_STREAM("RT:RAT: DIFF diff=" << diff << " d_e=" << dm.d_e);
                }
                dm.h_i = 0;
                */

                // publish the message
                //ROS_DEBUG_STREAM( "Publish frame_id=" << dm.frame_id << " relative_rad=" << dm.relative_rad );
                pub_dm.publish(dm);

                // Allow this closure.
                //if (a < distance_thr) {
                if (dm.calc1 > 90 && dm.calc1 < 110 && wait_cnt == 0) {
                    ROS_DEBUG_STREAM("RT:RAT: ALLOW a=" << a << " highest_id=" << highest_id << " curr_id=" << curr_id);
                    allow_old = highest_id;
                    allow_new = curr_id;
                } else if (wait_cnt == 0) {
                    ROS_DEBUG_STREAM("RT:RAT: JUMP calc1=" << dm.calc1 << " highest_id=" << highest_id << " curr_id=" << curr_id);

                    // FIXME: not same id?
                    wait_cnt = 50;
                }

                /*
                if ((float)dm.h_i < (float)distance_thr) {
                    ROS_DEBUG_STREAM("RT:RAT: ALLOW a=" << a << " highest_id=" << highest_id << " curr_id=" << curr_id);
                    allow_old = highest_id;
                    allow_new = curr_id;
                }
                */
                
                //ROS_DEBUG_STREAM("RT:RAT: GetDistance a=" << a << " b=" << b << " c=" << c << " d=" << d << " e=" << e << " f=" << f << " g=" << g);
            }
        }

        //double dt = (current_time - last_time).toSec();

        ratslam_ros::ViewTemplate vt;
        vt.header.stamp = current_time;
        vt.header.frame_id = "";
        vt.current_id = frame_id;
        vt.relative_rad = 0.0;

        // publish the message
        //ROS_DEBUG_STREAM( "Publish frame_id=" << vt.frame_id << " relative_rad=" << vt.relative_rad );
        pub_lv.publish(vt);

        // Allow loop closures.
        if (allow_old > 0 && allow_new > 0) {
            // FIXME: inside here? should be aware of all potential closures so it can build a map? or polluted?
            // Now that the new signature has been added to Pose network.
            // Go ahead and publish the matched signature.
            if (matched_id > 0) {
                frame_id = matched_id;
                matched_id = 0;
            }

            rtabmap_ros::AllowLoop srv;
            srv.request.oldId = allow_old;
            srv.request.newId = allow_new;
            if (client_allow.call(srv))
            {
                ROS_INFO_STREAM("RT:RAT: Allowed loop oldId=" << allow_old << " newId=" << allow_new);
            }
            else
            {
                ROS_ERROR("RT:RAT: Failed to call loop allowance service");
            }
            allow_new = 0;
            allow_old = 0;
        }

        // Remember last high for distance calcs.
        // FIXME: only update once moved on... somehow...
        if (highest_val > loop_thr) {
            last_high_id = highest_id;
        }

        // Reject loop closures.
        /*
        if (reject_loops && reject_old > 0 && reject_new > 0) {
            //
            int old_sig = 0;
            int new_sig = 0;
            std::vector<int>::iterator old_it = std::find(frames.begin(), frames.end(), (int)reject_old);
            if ((int)old_it[0] == (int)reject_old) {
                old_sig = old_it - frames.begin();
            }
            std::vector<int>::iterator new_it = std::find(frames.begin(), frames.end(), (int)reject_new);
            if ((int)new_it[0] == (int)reject_new) {
                new_sig = new_it - frames.begin();
            }
            //

            rtabmap_ros::RejectLoop srv;
            srv.request.oldId = reject_old;
            srv.request.newId = reject_new;
            if (reject_loop.call(srv))
            {
                ROS_INFO_STREAM("Rejected loop oldId=" << reject_old << " newId=" << reject_new);
            }
            else
            {
                ROS_ERROR("Failed to call loop rejection service");
            }
            reject_new = 0;
            reject_old = 0;
        }
        */

        last_time = current_time;
        if (wait_cnt > 0) --wait_cnt;

        //ros::spin();
        ros::spinOnce();
        r.sleep();
    }

}
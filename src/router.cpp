#include <inter_iit_uav_fleet/callbacks.h>
#include <fstream>

#include <std_msgs/String.h>
#include <inter_iit_uav_fleet/RouterData.h>

#define echo(x) std::cout << x << std::endl
#define sq(X) (X)*(X)
#define numQuads 3
#define numObjects 5

std::string mavName, curr_state, names[numQuads];
inter_iit_uav_fleet::RouterData routerData[numQuads - 1];
// int entries[numQuads - 1];

bool saveGPS = true;
bool received = false;
int id=-1, pubId = -1;
std::string gpsPath = "router/gps.txt";

struct locData{
    double lat;
    double lon;
    // bool publish;
};

int ids[numQuads][numObjects];
struct locData objects[numObjects];
int numEntries = 0;

void r1Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[0] = msg; }
void r2Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[1] = msg; }

void updateTable()
{
    for(int i = 0; i < obj_data.object_poses.size(); i++)
    {
        bool accept = false;
        for(int j = 0; j < numEntries; j++)
        {
            if(sq(objects[j].lat - obj_data.object_poses.at(i).position.x) + \
                sq(objects[j].lon - obj_data.object_poses.at(i).position.y) < loc_error)
            { accept = false; break; }
        }

        if(accept)
        {
            if(numEntries > 4) break;
            objects[numEntries].lat = obj_data.object_poses.at(i).position.x;
            objects[numEntries].lon = obj_data.object_poses.at(i).position.y;
            numEntries++;
        }
    }

    for(int i = 0; i < numQuads - 1; i++)
    {
        if(numEntries > 4) break;
        if(numEntries != routerData[i].id)
        {
            objects[numEntries].lat = routerData[i].position.x;
            objects[numEntries].lon = routerData[i].position.y;
            numEntries++;
        }
    }
}

void updateRouters(ros::Publisher *pub)
{
    inter_iit_uav_fleet::RouterData msg;
    if(routerData[0].id != numEntries || routerData[1].id != numEntries){
        msg.id = numEntries - 1;
        msg.position.x = objects[numEntries-1].lat;
        msg.position.y = objects[numEntries-1].lon; 
        msg.position.z = 0;
    }
    
    pub->publish(msg);
}

void saveData()
{
    std::ofstream file;
    file.open(gpsPath);
    file << std::to_string(numEntries) << "\n";
    for (int i = 0; i < numEntries; i++) file << std::to_string(objects[i].lat) + \
     " " + std::to_string(objects[i].lon) + "\n";
    file.close();
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "router");
    ros::NodeHandle nh, ph("~");

    ros::Rate loopRate(10);
    nh.getParam("verbose", verbose);
    nh.getParam("mav_name", mavName);

    nh.getParam("names/A", names[0]);
    nh.getParam("names/B", names[1]);
    nh.getParam("names/C", names[2]);
    
    for(int i=0; i<numQuads; i++)  if(mavName == names[i]) id = i+1;
    if(verbose) echo("Quad number:" << id);

    ros::NodeHandle routers[numQuads] = {"/" + names[id-1], "/" + names[(id)%numQuads], "/" + names[(id+1)%numQuads]};
    
    ros::Subscriber objSub = routers[0].subscribe("objects", 10, obj_cb_);
    // ros::Subscriber stateSub = routers[0].subscribe("curr_state", 10, state_cb_);
    // ros::Subscriber utmSub = routers[0].subscribe("utm_pose", 10, utm_pose_cb_);
    // ros::Subscriber odomSub = routers[0].subscribe("odometry", 10, mav_pose_cb_);
    // ros::Subscriber gpsSub = routers[0].subscribe("gps", 1, gpsCallback);

    ros::Subscriber subs[numQuads - 1] = {routers[1].subscribe("router/data", 10, r1Callback), routers[2].subscribe("router/data", 10, r2Callback)};

    ros::Publisher routerPub = ph.advertise<inter_iit_uav_fleet::RouterData>("data", 10);
    // ros::Publisher taskPub = routers[0].advertise<inter_iit_uav_fleet::TaskInfo>("task", 10);

    ros::ServiceClient terminator = nh.serviceClient<std_srvs::SetBool>("planner/stop");

    while(ros::ok())
    {
        ros::spinOnce();
        updateTable();
        updateRouters(&routerPub);
        if(numEntries == numObjects) break;
        loopRate.sleep();
    }
    saveData();

    return 0;
}

#include <inter_iit_uav_fleet/callbacks.h>
#include <fstream>
#include <future>

#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
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

int entries[numQuads];
struct locData objects[numObjects];
int lastEntry = -1;

void n1Callback(const std_msgs::Int16& msg){entries[0] = msg.data;}
void n2Callback(const std_msgs::Int16& msg){entries[1] = msg.data;}
void r1Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[0] = msg; }
void r2Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[1] = msg; }

void updateTable()
{
    // echo(obj_data.imageID);
    for(int i = 0; i < obj_data.object_poses.size(); i++)
    {
        bool accept = true;
        // if(verbose) echo("Processing detector values");
        for(int j = 0; j <= lastEntry; j++)
        {
            if((sq(objects[j].lat - obj_data.object_poses.at(i).position.x) + sq(objects[j].lon - obj_data.object_poses.at(i).position.y)) < loc_error)
            {   
                accept = false; 
                // if(verbose) echo("Rejected");
                break; 
            }
        }

        if(accept)
        {
            if(lastEntry == 3) break;
            lastEntry++;
            objects[lastEntry].lat = obj_data.object_poses.at(i).position.x;
            objects[lastEntry].lon = obj_data.object_poses.at(i).position.y;
            if(verbose) echo("Accepted(D): " << objects[lastEntry].lat << " " << " " << objects[lastEntry].lon << " at index: " << lastEntry);
        }
    }

    for(int i = 0; i < numQuads - 1; i++)
    {
        // if(verbose) echo("Processing router values");
        if(lastEntry == 3) break;
        if(lastEntry < routerData[i].id && entries[i] != -1)
        {
            lastEntry++;
            objects[lastEntry].lat = routerData[i].position.x;
            objects[lastEntry].lon = routerData[i].position.y;
            if(verbose) echo("Accepted(R): " << objects[lastEntry].lat << " " << " " << objects[lastEntry].lon << " at index: " << lastEntry);
        }
    }
}

void updateRouters(ros::Publisher *pub)
{
    inter_iit_uav_fleet::RouterData msg;

    if((lastEntry > entries[0] && entries[0] != 3)|| (lastEntry > entries[1] && entries[1] != 3))
    {
        int i = lastEntry - entries[0], j = lastEntry - entries[1];
        for(int k = 0; k < i; k++)
        {
            msg.id = lastEntry - k;
            msg.position.x = objects[lastEntry - k].lat;
            msg.position.y = objects[lastEntry - k].lon; 
            msg.position.z = 0;
            pub->publish(msg);
            if(verbose) echo("Published: " << objects[lastEntry-k].lat << " " << " " << objects[lastEntry-k].lon << " from index: " << lastEntry-k);
        }
        for(int k = 0; k < j; k++)
        {
            msg.id = lastEntry - k;
            msg.position.x = objects[lastEntry - k].lat;
            msg.position.y = objects[lastEntry - k].lon; 
            msg.position.z = 0;
            pub->publish(msg);
            if(verbose) echo("Published: " << objects[lastEntry-k].lat << " " << " " << objects[lastEntry-k].lon << " from index: " << lastEntry-k);
        }
    }
}

void saveData()
{
    std::ofstream file;
    file.open(gpsPath);
    file << std::to_string(lastEntry + 1) << "\n";
    for (int i = 0; i < lastEntry + 1; i++) file << std::to_string(objects[i].lat) + \
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
    ros::Subscriber dsubs[numQuads - 1] = {routers[1].subscribe("router/data", 10, r1Callback), routers[2].subscribe("router/data", 10, r2Callback)};
    ros::Subscriber nsubs[numQuads - 1] = {routers[1].subscribe("router/num", 10, n1Callback), routers[2].subscribe("router/num", 10, n2Callback)};

    ros::Publisher routerPub = ph.advertise<inter_iit_uav_fleet::RouterData>("data", 10);
    ros::Publisher numberPub = ph.advertise<std_msgs::Int16>("num", 10);

    ros::ServiceClient terminator = nh.serviceClient<std_srvs::SetBool>("planner/stop");

    std_msgs::Int16 msg;
    entries[0] = entries[1] = -1;
    
    while(ros::ok())
    {
        ros::spinOnce();
        msg.data = lastEntry;
        numberPub.publish(msg);
        updateTable();
        updateRouters(&routerPub);
        if(lastEntry + 1 == numObjects - 1) break;
        loopRate.sleep();
    }

    std_srvs::SetBool srv; srv.request.data = false;
    saveData();
    echo("Saved gps location data");
    while(!srv.response.success) terminator.call(srv);

    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <queue>
#include <string>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

class MazeReactiveSolver : public rclcpp::Node
{
public:
    MazeReactiveSolver() : Node("Solver")
    {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        dir = {0,1};
        found_wall = false;
        is_in_target= false;
        possible_directions = {"right", "down", "left", "up"};
        possible_vectors = {{1,0}, {0,-1}, {-1,0}, {0,1}};

        while (!client_->wait_for_service(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service '/move_command'...");
        }
        
        std::string init = "up";
        RCLCPP_INFO(this->get_logger(), init.c_str());
        send_move_request(init);
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    std::array<int,2> dir;
    bool found_wall;
    std::vector<std::string> possible_directions;
    std::vector<std::array<int,2>> possible_vectors;
    bool is_in_target;

    void send_move_request(const std::string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto future_result = client_->async_send_request(request,
            std::bind(&MazeReactiveSolver::process_response, this, std::placeholders::_1));
    }

    void process_response(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture response_future)
    {
        auto response = response_future.get();
        if(is_in_target){
          RCLCPP_INFO(this->get_logger(), "Found target!");
          rclcpp::shutdown();
        }
        if (response->success) {
          std::vector<std::string> info = {response->right, response->down, response->left, response->up};
          if(!found_wall){
            if(info[3]=="b"){
              dir = clockwise(dir);
              found_wall=true;
              std::string move = "right";
              RCLCPP_INFO(this->get_logger(), move.c_str());
              send_move_request(move);
            } else {
              std::string move = "up";
              RCLCPP_INFO(this->get_logger(), move.c_str());
              send_move_request(move);
            };
          } else {
            int pos_arr = 0;
            int new_dir_pos = 0;
            int target = -1;
            for(int i=0; i < 4; i++){
              if(dir == possible_vectors[i]){
                pos_arr = i;
              };
              if(info[i]=="t"){
                target = i;
                break;
              }
            };
            int l = (pos_arr + 3)%4;
            int r = (pos_arr + 1)%4;
            if(target==-1){
              if(info[l]=="b"){
                if(info[pos_arr]=="b"){
                  dir = clockwise(dir);
                  new_dir_pos = r;
                } else {
                  new_dir_pos = pos_arr;
                };
              } else if (info[l]=="f"){
                dir = anti_clockwise(dir);
                new_dir_pos = l;
              };
              RCLCPP_INFO(this->get_logger(), possible_directions[new_dir_pos].c_str());
            } else {
              new_dir_pos = target;
              is_in_target = true;
            }
            send_move_request(possible_directions[new_dir_pos]);
          }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to move.");
            rclcpp::shutdown();
        }
    }

    std::array<int,2> clockwise(const std::array<int,2> &arr){
      std::array<int,2> rot = {arr[1],-arr[0]};
      return rot;
    }

    std::array<int,2> anti_clockwise(const std::array<int,2> &arr){
      std::array<int,2> rot = {-arr[1],arr[0]};
      return rot;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeReactiveSolver>();
    rclcpp::spin(node);
    return 0;
}
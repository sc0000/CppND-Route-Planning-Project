#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// Calculate h-value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Add neighbor nodes to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        open_list.push_back(neighbor);
        neighbor->visited=true;
    }
}


// Sort the open list to start with the node with the lowest h-value. Create a pointer to this node and remove it from the list.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* node1, const RouteModel::Node* node2) -> bool {
        return (node1->g_value + node1->h_value < node2->g_value + node2->h_value);
    });
    RouteModel::Node* next_node = open_list[0];
    open_list.erase(open_list.begin());
    return next_node;
}


// This method gets called once the end node is reached. It follows the chain of parents back to the starting nodes, 
// adding them to the path_found vector and adding up the distances. Before return, the vector has to be reversed.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);    
      	distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    distance *= m_Model.MetricScale();
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


// A star search algorithm.
void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    open_list.push_back(start_node);
    while (!open_list.empty()) {
        RouteModel::Node *current_node = NextNode();
        if(current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);        
    }
}
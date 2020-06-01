#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();
    for (auto neighbour : current_node->neighbors){
        neighbour->parent = current_node;
        neighbour->g_value = current_node->g_value + neighbour->distance(*current_node);
        neighbour->h_value = CalculateHValue(neighbour);
        neighbour->visited = true;
        open_list.push_back(neighbour);
    }

}


bool RoutePlanner::CompareNodes(const RouteModel::Node *n1, const RouteModel::Node *n2) {
  float f1 = n1->g_value + n1->h_value;
  float f2 = n2->g_value + n2->h_value;
  return f1 > f2; 
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::CompareNodes);
    auto current = open_list.back();
    open_list.pop_back();
    return current;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    auto path_node = current_node;
    while(path_node != start_node){
        distance += path_node->distance(*(path_node->parent));
        path_found.push_back(*path_node);
        path_node = path_node->parent;
    }
    path_found.push_back(*start_node);
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}
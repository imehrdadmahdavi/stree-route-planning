#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage.
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Using the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Storing the found nodes in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);

}


// CalculateHValue method calculates the h value for the given node.
// - It uses the distance to the end_node.
// - Note: Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
// - It uses the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors,it then sets the parent, the h_value, the g_value. 
// - It also used the CalculateHValue to implement the h-Value calculation.
// - Finally, for each node in current_node.neighbors, it adds the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();
    float current_g = current_node->g_value;
    current_node->visited = true;

    for (int i = 0; i < current_node->neighbors.size(); i++) {

        RouteModel::Node *neighbor = current_node->neighbors[i];
        
        neighbor->parent = current_node;
        neighbor->g_value = current_g + neighbor->distance(*current_node);
        neighbor->h_value = CalculateHValue(neighbor);

        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


// NextNode method sorts the open list and return the next node.
// - It sorts the open_list according to the sum of the h value and g value.
// - It then creates a pointer to the node in the list with the lowest sum
//   and removes that node from the open_list.
// - Finaly it returns the pointer.

float Compare(RouteModel::Node* a, RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;

    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {

    sort(this->open_list.begin(), this->open_list.end(), Compare);
    RouteModel::Node* next = open_list.back();

    return next;
}


// ConstructFinalPath method returns the final path found from the A* search.
// - This method takes the current (final) node as the argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, it adds the distance from the node to its parent to the distance variable.
// - The returned vector will be reveres to be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->x != this->start_node->x && current_node->y != this->start_node->y){
        
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// The A* Search algorithm calculates the path from the start to the end node. It also calcualtes the distace between them.
// - This function uses the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - It then uses the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, the function uses the ConstructFinalPath method to return the final path that was found.
// - Finally, it stores the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    
    RouteModel::Node *current_node = nullptr;
    this->open_list.push_back(this->start_node);

    while(this->open_list.size() > 0) {

        current_node = this->NextNode();
        this->open_list.pop_back();

        if (current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        AddNeighbors(current_node);           
    }
}
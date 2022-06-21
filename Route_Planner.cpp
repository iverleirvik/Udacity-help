#include "route_planner.h"
#include <algorithm>
#include <cassert>
#include <iostream>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}




// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    float hValue = node->distance(*this->end_node); //Finds the distance between the argument node and end node.

    return hValue;

}




// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->RouteModel::Node::FindNeighbors();    // This function finds the neighbors of current_node

    for (RouteModel::Node * naboNode : current_node->neighbors) // The point here is to iterate through all the neigboors of 
    {                                                           // current node, which we just found. Then we set the attributes of the
        naboNode->parent = current_node;                        // different neighbors to the right values.
        naboNode->g_value = current_node->g_value + current_node->distance(*naboNode);
        naboNode->h_value = this->CalculateHValue(naboNode);    // Here we calculate the g and h value which wil be used for assessing
        this->open_list.push_back(naboNode);                    // if the different nodes is the next step for our path.
        naboNode->visited = true;

    }
}




// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.


RouteModel::Node *RoutePlanner::NextNode() {    // This is the function where I am having the most trouble I think.
                                                // I will go through what I am thinking som you can understand.
    float lowestSumValue;   // Variable used for storing the lowest sum og h and g value.
    RouteModel::Node* ptrLowestSum; // declare a object which will be used to store the address of the node with the lowest sum.
    int indexOfNode;    // Declaration of variable used to remember the position of the right node in the open_list.
    int i = -1;  // Initializing an iterator.

    for (RouteModel::Node* node : this->open_list)  // Iterating over the nodes in the open_list.
    {
        i = i + 1;    // Adding 1 for every iteration. Used to keep track of position in open_list.
        float sumValue = node->g_value + node->h_value; // Calculate the sum which will determine the appropriate neighboring node.                                      
        if (sumValue < lowestSumValue)  // if-statement for checking if the current sum is lower than the lowest sum for this session.
        {
            lowestSumValue = sumValue;  // Setting the lowest sum.
            ptrLowestSum = node;    // Setting the pointer to the node with the lowest sum.
            indexOfNode = i;    // The index in the open_list vector for the node with the lowest sum.
            
        }
    }

    this->open_list.erase(this->open_list.begin()+indexOfNode); // Removing the node which is chosen as the nextstep from the open_list.
    return ptrLowestSum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    RouteModel::Node* previousNode = current_node;

    while(previousNode)     // previousNode != RoutePlanner::start_node
    {
        path_found.insert(path_found.begin(), *previousNode);
        //path_found.push_back(*previousNode);

        if(previousNode->parent) this->distance += previousNode->distance(*previousNode->parent);
        previousNode = previousNode->parent;
    }

    this->distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.


    return path_found;
}




// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.


void RoutePlanner::AStarSearch() {  // The AStarSearch fucntion did not pass the test. I only received segmentation fault
    RouteModel::Node *current_node = nullptr;   // so not any other feedback to troubleshoot from.
                                                // I will here go through the function and help clarify what ive been thinking.
    // TODO: Implement your solution here.

    current_node = this->start_node;    // First we set the current_node to the start_node
    this->start_node->visited = true;   // set the attribute to true.
    this->open_list.push_back(this->start_node); 

    while (current_node != this->end_node)  // Checks if we have reached the end node.
    {
        current_node = this->NextNode();    // Finds the next node.
        if (current_node != this->end_node) 
        {                                       // If the current_node isnt the end node we can
            this->AddNeighbors(current_node);   // find the neighbors of that node.
        }
        
    }

    assert (current_node == this->end_node);    // Checks if we have reached the end node, if not it fails.

    this->m_Model.path = this->ConstructFinalPath(current_node);    // Constructs our final path
}

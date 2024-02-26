# import necessary libraries
import io
import math
import numpy as np
import copy
import time
import argparse

##### THE FOLLOWING HELPER FUNCTIONS AND CLASSES COME FROM evaluateShared.py AND ARE SIMPLY UTILIZED TO PULL IN AND ORGANIZE THE LOAD DATA IN A MANNER THAT IS CONSISTENT WITH THE OVERALL SYSTEM #####

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def distanceBetweenPoints(p1, p2):
    xDiff = p1.x - p2.x
    yDiff = p1.y - p2.y
    return math.sqrt(xDiff*xDiff + yDiff*yDiff)
    
class Load:
    def __init__(self, id, pickup, dropoff):
        self.id = id
        self.pickup = pickup
        self.dropoff = dropoff
        
class VRP:
    def __init__(self, loads):
        self.loads = loads
        
def loadProblemFromFile(filePath):
    f = open(filePath, "r")
    problemStr = f.read()
    f.close()
    return loadProblemFromProblemStr(problemStr)

def getPointFromPointStr(pointStr):
    pointStr = pointStr.replace("(","").replace(")","")
    splits = pointStr.split(",")
    return Point(float(splits[0]), float(splits[1]))

def loadProblemFromProblemStr(problemStr):
    loads = []
    buf = io.StringIO(problemStr)
    gotHeader = False
    while True:
        line = buf.readline()
        if not gotHeader:
            gotHeader = True
            continue
        if len(line) == 0:
            break
        line = line.replace("\n", "")
        splits = line.split()
        id = splits[0]
        pickup = getPointFromPointStr(splits[1])
        dropoff = getPointFromPointStr(splits[2])
        loads.append(Load(id, pickup, dropoff))
    return VRP(loads)


##### THE FOLLOWING SECTION CONTAINS MY CUSTOM CLASSES, FUNCTIONS, ALGORITHMS, AND PROGRAM FOR SOLVING THIS VERSION OF THE VEHICLE ROUTING PROBLEM #####


def generateDistanceArrays(VehicleRoutingProblem):
    # initialize arrays which will contain the pairwise distances between the origin, pickup locations, and dropoff locations
    num_loads = len(VehicleRoutingProblem.loads)
    distance_from_origin_to_pickups = np.empty(num_loads)
    distance_from_pickups_to_dropoffs = np.empty(num_loads)
    distance_from_dropoffs_to_pickups = np.empty((num_loads, num_loads))
    distance_from_dropoffs_to_origin = np.empty(num_loads)

    # instantiate the origin
    origin = Point(0.0, 0.0)

    # for each load, compute and store the distance between its pickup and dropoff locations and the origin
    for loadIndex, load in enumerate(VehicleRoutingProblem.loads):
        distance_from_origin_to_pickups[loadIndex] = distanceBetweenPoints(origin, load.pickup)
        distance_from_dropoffs_to_origin[loadIndex] = distanceBetweenPoints(load.dropoff, origin)

    # for each load, compute and store both the distance required to complete the load and the pairwise distance between the load's dropoff location and all other loads' pickup locations
    for loadOneIndex, loadOne in enumerate(VehicleRoutingProblem.loads):
        distance_from_pickups_to_dropoffs[loadOneIndex] = distanceBetweenPoints(loadOne.pickup, loadOne.dropoff)
        for loadTwoIndex, loadTwo in enumerate(VehicleRoutingProblem.loads):
            distance_from_dropoffs_to_pickups[loadOneIndex][loadTwoIndex] = distanceBetweenPoints(loadOne.dropoff, loadTwo.pickup) if loadOneIndex != loadTwoIndex else np.inf

    return distance_from_origin_to_pickups, distance_from_pickups_to_dropoffs, distance_from_dropoffs_to_pickups, distance_from_dropoffs_to_origin


class costEvaluator:
    # initialize the evaluator class by internally copying and storing the distance arrays for a given VRP
    def __init__(self,
                 distance_from_origin_to_pickups,
                 distance_from_pickups_to_dropoffs,
                 distance_from_dropoffs_to_pickups,
                 distance_from_dropoffs_to_origin):
        self.distance_from_origin_to_pickups = copy.deepcopy(distance_from_origin_to_pickups)
        self.distance_from_pickups_to_dropoffs = copy.deepcopy(distance_from_pickups_to_dropoffs)
        self.distance_from_dropoffs_to_pickups = copy.deepcopy(distance_from_dropoffs_to_pickups)
        self.distance_from_dropoffs_to_origin = copy.deepcopy(distance_from_dropoffs_to_origin)

    # define a class method to calculate the distance traversed by an individual route
    def calculateRouteDistance(self, route):
        # initialize the route distance to be the distance from the origin to the first pickup location visited
        route_distance = self.distance_from_origin_to_pickups[route[0]-1]

        if len(route) > 1:
            # for each intermediate load in the route, add the distance required to complete that delivery and then to reach the next load's pickup location to the overall route distance
            for load_index in range(len(route)-1):
                route_distance += self.distance_from_pickups_to_dropoffs[route[load_index]-1]
                route_distance += self.distance_from_dropoffs_to_pickups[route[load_index]-1][route[load_index+1]-1]
            # once the final load has been picked up, add the distance required to complete that delivery and then to return to the origin to the overall route distance
            route_distance += self.distance_from_pickups_to_dropoffs[route[load_index+1]-1]
            route_distance += self.distance_from_dropoffs_to_origin[route[load_index+1]-1]

        # if there is only one load in the route, simply add the distance required to complete that delivery and then to return to the origin to the overall route distance
        elif len(route) == 1:
            route_distance += self.distance_from_pickups_to_dropoffs[route[0]-1]
            route_distance += self.distance_from_dropoffs_to_origin[route[0]-1]
        
        return route_distance
    
    # define a class method to calculate the total cost associated with a specific VRP solution
    def calculateTotalCost(self, routes):
        # initialize the total cost
        total_cost = 0
        
        # for each route in the VRP solution calculate the cost of that individual route and add it to the total cost of the solution
        for route in [route_tuple[0] for route_tuple in routes]:
            # when a new driver is added, we must add 500 to the total cost
            total_cost += 500

            # add the cost incurred by the route's distance to the total cost
            total_cost += self.calculateRouteDistance(route)

        return total_cost


def runNearestNeighborAlgorithm(num_loads,
                                distance_from_origin_to_pickups,
                                distance_from_pickups_to_dropoffs,
                                distance_from_dropoffs_to_pickups,
                                distance_from_dropoffs_to_origin):
    # copy and store the distance arrays for a given VRP to avoid inappropriately altering these arrays in the global scope
    temp_distance_from_origin_to_pickups = copy.deepcopy(distance_from_origin_to_pickups)
    temp_distance_from_pickups_to_dropoffs = copy.deepcopy(distance_from_pickups_to_dropoffs)
    temp_distance_from_dropoffs_to_pickups = copy.deepcopy(distance_from_dropoffs_to_pickups)
    temp_distance_from_dropoffs_to_origin = copy.deepcopy(distance_from_dropoffs_to_origin)

    # initialize the routes array, the number of loads delivered, and the total cost
    routes = []
    num_loads_delivered = 0
    total_cost = 0
    # iterate through the routing algorithm until all loads have been delivered
    while num_loads_delivered < num_loads:
        # when a new driver is added, we must add 500 to the total cost and reinitialize the route and drive time for that driver
        total_cost += 500
        route = []
        current_drive_time = 0

        # let the first load delivered by a new driver be the one whose pickup location has the minimum distance to the origin (out of those that have not been delivered yet)
        current_load_index = np.argmin(temp_distance_from_origin_to_pickups)
        route.append(current_load_index+1)
        current_drive_time += temp_distance_from_origin_to_pickups[current_load_index] + \
                              temp_distance_from_pickups_to_dropoffs[current_load_index]
        num_loads_delivered += 1

        # once a load has been delivered, set the distance to its pickup location from any other location to infinity so that it is never selected first
        temp_distance_from_origin_to_pickups[current_load_index] = np.inf
        for load_idx in range(num_loads):
            temp_distance_from_dropoffs_to_pickups[load_idx][current_load_index] = np.inf

        # continue adding loads to the current driver's route following a Nearest Neighbor approach until the maximum drive time stopping condition is met
        terminate_route = False
        while not terminate_route:
            # sort the next loads to consider delivering based on their pickup locations' distances from the current load's dropoff location
            next_load_indices = np.argsort(temp_distance_from_dropoffs_to_pickups[current_load_index])
            # let the nearest load whose marginal delivery time allows the current route's cumulative drive time to satisfy the maxmimum allowable drive time constraint be the next load delivered
            for next_load_index in next_load_indices:
                next_drive_time = current_drive_time + \
                                  temp_distance_from_dropoffs_to_pickups[current_load_index][next_load_index] + \
                                  temp_distance_from_pickups_to_dropoffs[next_load_index] + \
                                  temp_distance_from_dropoffs_to_origin[next_load_index]
                
                # if the cumulative time it would take to deliver this selected load and return to the origin is still less than the maxmimum allowable drive time, let this be the next load added to the current driver's route
                # (note that we did not need to include this additional check when selecting the first load since every individual load is guaranteed to be deliverable within the time constraint)
                if next_drive_time <= 12*60:
                    route.append(next_load_index+1)
                    current_drive_time += temp_distance_from_dropoffs_to_pickups[current_load_index][next_load_index] + \
                                          temp_distance_from_pickups_to_dropoffs[next_load_index]
                    num_loads_delivered += 1

                    # once a load has been delivered, set the distance to its pickup location from any other location to infinity so that it is never selected
                    # (note that "never selected" does not mean it can never be proposed as the next load to consider delivering, but, since its distance is now set to infinity, trying to deliver it will always exceed the maximum allowable drive time, and, therefore, it will never actually be selected and added to another driver's route)
                    temp_distance_from_origin_to_pickups[next_load_index] = np.inf
                    for load_idx in range(num_loads):
                        temp_distance_from_dropoffs_to_pickups[load_idx][next_load_index] = np.inf
                    
                    # let the next load become the current load and repeat this step of the algorithm
                    current_load_index = next_load_index
                    break

                # if the proposed next load to deliver causes the current driver's drive time to exceed the maximum allowable limit, do not add it to the current route
                elif next_drive_time > 12*60:
                    # if the proposed next load to deliver causes the current driver's drive time to be infinite, this means that all of the undelivered loads have been considered already and we should terminate the route and have the driver return to the origin
                    if (next_drive_time == np.inf):
                        terminate_route = True
                        break
                    # if not all of the undelivered loads have been considered, proceed to the evaluate the viability of the next nearest one
                    else:
                        pass

        # add the distance to the origin to the current drive time and add this new current drive time to the total cost
        # (note that this drive time will not exceed the maximum limit since that was required for the proposed load to be added to the route via the conditional statements above)
        current_drive_time += temp_distance_from_dropoffs_to_origin[current_load_index]
        total_cost += current_drive_time

        # add this route to the list of driver routes and also store the route's drive time for posterity
        routes.append((route, current_drive_time))
        
    return routes, total_cost


def runTwoOptAlgorithm(routes, total_cost, cost_evaluator):
    # select a route at random out of all possible routes whose lengths are greater than one
    selected_route_index = np.random.choice([index for index, route in enumerate(routes) if len(route[0]) > 1])
    selected_route = routes[selected_route_index][0]

    # store the travel distance associated with the selected route
    selected_route_distance = routes[selected_route_index][1]

    # generate two random indices that will delimit the slice of loads of the selected route that will be reversed, ensuring that the indices are in ascending order
    i, j = np.sort(np.random.randint(len(selected_route), size=2))
    if i != j:
        # let the proposed route be identical to the selected route except with the slice of loads defined above reversed
        proposed_route = copy.deepcopy(selected_route)
        proposed_route[i:j] = selected_route[i:j][::-1]

        # calculate the travel distance associated with the proposed route
        proposed_route_distance = cost_evaluator.calculateRouteDistance(proposed_route)

        # if the proposed route achieves a shorter distance (i.e. is more optimal) than the current version of the selected route, accept the change and update the VRP solution and associated total cost
        if proposed_route_distance < selected_route_distance:
            routes[selected_route_index] = (proposed_route, proposed_route_distance)
            total_cost = total_cost - selected_route_distance + proposed_route_distance

        return routes, total_cost
    
    # if the indices delimiting the slice of loads of the selected route that will be reversed are identical, simply return the current VRP solution and associated total cost
    elif i == j:
        return routes, total_cost


def main():
    # save the program's start time and parse the command line arguments being received
    start_time = time.time()
    argParser = argparse.ArgumentParser()
    argParser.add_argument("inputFile", help="The text file path describing a VRP.")
    args = argParser.parse_args()

    # load in the Vehicle Routing Problem from a given text file path
    VehicleRoutingProblem = loadProblemFromFile(args.inputFile)

    # determine the number of loads that need to be delivered and generate arrays containing the pairwise distances between the origin, pickup locations, and dropoff locations
    num_loads = len(VehicleRoutingProblem.loads)
    distance_from_origin_to_pickups, distance_from_pickups_to_dropoffs, distance_from_dropoffs_to_pickups, distance_from_dropoffs_to_origin = generateDistanceArrays(VehicleRoutingProblem)

    # instantiate an evaluator that can quickly and efficiently calculate individual route and total solution costs
    cost_evaluator = costEvaluator(distance_from_origin_to_pickups,
                                              distance_from_pickups_to_dropoffs,
                                              distance_from_dropoffs_to_pickups,
                                              distance_from_dropoffs_to_origin)
    
    # run the Nearest Neighbors algorithm to heuristically determine a strong initial solution
    best_routes, total_cost = runNearestNeighborAlgorithm(num_loads,
                                                          distance_from_origin_to_pickups,
                                                          distance_from_pickups_to_dropoffs,
                                                          distance_from_dropoffs_to_pickups,
                                                          distance_from_dropoffs_to_origin)

    # set a seed for reproducibility and run the 2-opt algorithm until a specified time limit for the program's runtime is reached
    # (note that, during experimentation, it was empirically found that this random 2-opt swapping procedure did not yield significant improvements over the Nearest Neighbor algorithm's solution regardless of the time limit, therefore, the time limit has been dramatically reduced to avoid incurring excessive and unproductive runtimes)
    np.random.seed(42)
    time_limit = 0.25
    while time.time() - start_time < time_limit:
        best_routes, total_cost = runTwoOptAlgorithm(best_routes, total_cost, cost_evaluator)

    # write the VRP solution to stdout
    for route in best_routes:
        print(route[0])


if __name__ == "__main__":
    main()
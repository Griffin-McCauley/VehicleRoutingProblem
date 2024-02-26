# Vehicle Routing Problem
### Griffin McCauley

## Introduction

The Vehicle Routing Problem (VRP) is a generalization of the Travelling Salesman Problem (TSP) that involves determining the optimal design of routes for a fleet of vehicles to follow such that all loads within a defined list are completed subject to a set of logistical constraints.

In this specific version of the VRP, each load is defined as a pair of points ((x1, y1), (x2, y2)) in the Cartesian plane where the first point represents a pickup location for the delivery and the second point represents the dropoff location for the delivery. Therefore, this variant would be considered a Vehicle Routing Problem with Pickup and Delivery (VRPPD).

By definition, a delivery is completed by traveling to the pickup location, picking up the load, traveling to the dropoff, and dropping off the load, suggesting that, in this instance, the vehicle capacities should be considered to be one load and that loads must be delivered immediately before picking up any other loads.

In this specific problem, the number of vehicles and drivers that can be utilized to deliver the assigned loads is unbounded, but no driver's shift can exceed 12 hours in duration before returning to the depot located at the origin (0,0). One unit of distance in this plane can be traveresed in one minute, so an individual driver's total route distance must be less than 12*60=720 units in length.

The total cost function attempting to be optimized is defined as `500*number_of_drivers + total_number_of_driven_minutes`, so the goal of this exercise is to produce a program that yields a solution that has a low total cost within a reasonable amount of time (<30 seconds).

(Note that no problems provided here will contain more than 200 loads and all loads will be possible to complete within a single 12-hour driving shift. Therefore, all problems will be solvable and the program devised does not need to assess solution feasibility.)

## Methodology

The program implemented in `Griffin_McCauley_VRP.py` consists of an intial solution being generated via a Nearest Neighbor heuristic approach followed by a secondary, local search process employing 2-Opt optimization.

### Nearest Neighbor Heuristic

This heuristic can be used to produce a reasonable initial solution by selecting the nearest neighbor to the current location as the next load to deliver. In the context of this specific problem space, this means selecting the first load to deliver as the one whose pickup location is closest to the origin and subsequently selecting the nearest neighbor to the current load's dropout location as the next load to deliver. A route is extended until the driver's shift (including the time it takes to return to the origin) threatens to exceed the maximum allowable drive time at which point the route is terminated and the driver returns to the origin.

Since each driver incurs an additional cost of 500, I included an extra provision in my implementation of the algorithm that selects the next load delivered to be the nearest neighbor whose marginal delivery time allows the current driver's cumulative drive time to satisfy the maximum allowable drive time constraint rather than only considering the top nearest neighbor, thus allowing for the program to extend each driver's route for as long as possible before returning to the origin.

### 2-Opt Optimization

After an initial solution is determined via the Nearest Neighbor algorithm described above, a local search 2-Opt optimization approach is adopted to try to iterate towards a more optimal solution. The goal of this method is to further improve the performance of the initial solution by pseudo-randomly permuting the loads within a given route to try to identify more efficient ways to traverse the route. Specifically, this technique involves randomly selecting an individual route from the set of routes comprising an initial solution and reversing a random slice of loads within that route. The length of this newly proposed route is then compared to the length of the initial route, and, if it is shorter (i.e. more efficient and optimal), this proposed route is merged into the solution as the new best solution.

Theoretically, this approach can be iteratively applied indefinitely, so, in practice, one would need to simply define a certain number of iterations to run it before stopping. In the case of this problem set up, however, since it is explicitly stated that programs should run no longer than 30 seconds, the implementation found in `Griffin_McCauley_VRP.py` defines a runtime limit that allows the 2-Opt optimization to continue running until that time limit is reached.

(Note that, while this technique should allow for the program to eventually explore all route permutations (of those defined by the initially determined solution), the random slice and reversal process can be highly inefficient if the original solution is already quite good, and, through experiments on the training set of problems found in the `trainingProblems` folder, it was empirically found that this secondary optimization did not produce significant improvements over the Nearest Neighbor heuristic's initial performance, so the time limit this secondary, local search optimization is allowed to run for is dramatically reduced to avoid incurring excessive and unproductive runtimes.)

### Other Approaches

Other metaheuristic-based approaches such as Genetic Algorithms, Tabu Search, Simulated Annealing and Adaptive Large Neighborhood Search (ALNS) have also been demonstrated to be highly effective at solving VRPs and are very active research areas in the field of applied mathematics and operations research, so, given more time, exploring these more advanced and sophisticated techniques would be a natural progression from the currently developed program.

## Usage

This program can be executed for a specific VRP via the command line by running `python3 Griffin_McCauley_VRP.py {path_to_problem}`.

Evaluating the program over all of the training problems provided in the `trainingProblems` folder can be achieved by running `python3 evaluateShared.py --cmd "python3 Griffin_McCauley_VRP.py" --problemDir trainingProblems`.

(Note that the only libraries utilized by this program are `io`, `math`, `numpy`, `copy`, `time`, and `argparse`. Most IDEs come with these libaries pre-installed, but, if they are not present in your current environment, please install them prior to running the program.)

## Evaluation Results

The program described above and implemented in Python in `Griffin_McCauley_VRP.py` is able to achieve an average total cost of **47945.39** when evaluated on the problems contained within the `trainingProblems` folder of this repository.

## References

1. https://iopscience.iop.org/article/10.1088/1742-6596/2421/1/012027/pdf

This article provided the initial inspiration to adopt a Nearest Neighbor algorithm approach.

2. https://medium.com/@writingforara/solving-vehicle-routing-problems-with-python-heuristics-algorithm-2cc57fe7079c

This blog post encouraged the pursuit of layering an additional 2-Opt optimization approach on top of the Nearest Neighbor heuristic solution.

Other resources that were beneficial for gaining a deeper understanding of the problem space can be found below:

3. https://en.wikipedia.org/wiki/Vehicle_routing_problem

4. https://www.sciencedirect.com/science/article/abs/pii/S0305054802000515
   
5. https://medium.com/rideos/tabu-search-for-the-vehicle-routing-problem-b1fd993f4301
   
6. https://www.hindawi.com/journals/mpe/2019/2358258/
   
7. https://dial.uclouvain.be/memoire/ucl/en/object/thesis%3A4615/datastream/PDF_01/view

8. https://d-nb.info/1072464683/34

9. https://www.intechopen.com/online-first/87515

10. https://en.wikipedia.org/wiki/2-opt

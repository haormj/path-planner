package hybridastar

import "container/heap"

type Algorithm struct {
}

func (a *Algorithm) HybridAStar(start *Node3D,
	goal *Node3D,
	nodes3D []*Node3D,
	nodes2D []*Node2D,
	width int32,
	height int32,
	configurationSpace *CollisionDetection,
	dubinsLookup []float32,
	visualization *Visualize) []*Node3D {

	return nil
}

func AStar(start *Node2D,
	goal *Node2D,
	nodes2D []*Node2D,
	width,
	height int32,
	configurationSpace *CollisionDetection) float32 {
	// PREDECESSOR AND SUCCESSOR INDEX
	var iPred, iSucc int32
	var newG float32

	// reset the open and closed list
	for i := 0; i < int(width*height); i++ {
		nodes2D[i].Reset()
	}

	// VISUALIZATION DELAY
	// ros::Duration d(0.001);

	O := &Node2DMinHeap{}
	heap.Init(O)

	// update h value
	start.UpdateH(goal)
	// mark start as open
	start.Open()
	// push on priority queue
	heap.Push(O, start)
	iPred = start.SetIdx(width)
	nodes2D[iPred] = start

	// NODE POINTER
	var nPred *Node2D
	var nSucc *Node2D

	// continue until O empty
	for O.Len() != 0 {
		// pop node with lowest cost from priority queue
		nPred = heap.Pop(O).(*Node2D)
		// set index
		iPred = nPred.SetIdx(width)

		// _____________________________
		// LAZY DELETION of rewired node
		// if there exists a pointer this node has already been expanded
		if nodes2D[iPred].IsClosed() || !nodes2D[iPred].IsOpen() {
			// pop node from the open list and start with a fresh node
			continue
		}

		// _________________
		// EXPANSION OF NODE
		// add node to closed list
		nodes2D[iPred].Close()
		nodes2D[iPred].Discover()

		// RViz visualization
		// if visualization2D {
		// 	visualization.publishNode2DPoses(*nPred)
		// 	visualization.publishNode2DPose(*nPred)
		// 	//        d.sleep();
		// }

		// _________
		// GOAL TEST
		if nPred.Equal(goal) {
			return nPred.GetG()
		}

		// ____________________
		// CONTINUE WITH SEARCH
		// _______________________________
		// CREATE POSSIBLE SUCCESSOR NODES
		for i := 0; i < int(node2dDir); i++ {
			// create possible successor
			nSucc = nPred.CreateSuccessor(int32(i))
			// set index of the successor
			iSucc = nSucc.SetIdx(width)

			// ensure successor is on grid ROW MAJOR
			// ensure successor is not blocked by obstacle
			// ensure successor is not on closed list
			if nSucc.IsOnGrid(width, height) && configurationSpace.IsTraversable(nSucc) && !nodes2D[iSucc].IsClosed() {
				// calculate new G value
				nSucc.UpdateG()
				newG = nSucc.GetG()

				// if successor not on open list or g value lower than before put it on open list
				if !nodes2D[iSucc].IsOpen() || newG < nodes2D[iSucc].GetG() {
					// calculate the H value
					nSucc.UpdateH(goal)
					// put successor on open list
					nSucc.Open()
					nodes2D[iSucc] = nSucc
					heap.Push(O, nodes2D[iSucc])
				}
			}
		}
	}
	// return large number to guide search away
	return 1000
}


//###################################################
//                                         COST TO GO
//###################################################
func UpdateH(start *Node3D, 
	goal *Node3D , 
	nodes2D []*Node2D , 
	dubinsLookup []float32 , 
	width,  height int32, 
	configurationSpace *CollisionDetection) {
	var dubinsCost float32 = 0;
	var  reedsSheppCost float32 = 0;
	var twoDCost float32 = 0;
	var  twoDoffset  float32= 0;
  
	// if dubins heuristic is activated calculate the shortest path
	// constrained without obstacles
	if dubins {
	  ompl::base::DubinsStateSpace dubinsPath(Constants::r);
	  State* dbStart = (State*)dubinsPath.allocState();
	  State* dbEnd = (State*)dubinsPath.allocState();
	  dbStart->setXY(start.getX(), start.getY());
	  dbStart->setYaw(start.getT());
	  dbEnd->setXY(goal.getX(), goal.getY());
	  dbEnd->setYaw(goal.getT());
	  dubinsCost = dubinsPath.distance(dbStart, dbEnd);
	}
  
	// if reversing is active use a
	if (Constants::reverse && !Constants::dubins) {
	  //    ros::Time t0 = ros::Time::now();
	  ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
	  State* rsStart = (State*)reedsSheppPath.allocState();
	  State* rsEnd = (State*)reedsSheppPath.allocState();
	  rsStart->setXY(start.getX(), start.getY());
	  rsStart->setYaw(start.getT());
	  rsEnd->setXY(goal.getX(), goal.getY());
	  rsEnd->setYaw(goal.getT());
	  reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
	  //    ros::Time t1 = ros::Time::now();
	  //    ros::Duration d(t1 - t0);
	  //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
	}
  
	// if twoD heuristic is activated determine shortest path
	// unconstrained with obstacles
	if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
	  //    ros::Time t0 = ros::Time::now();
	  // create a 2d start node
	  Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
	  // create a 2d goal node
	  Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
	  // run 2d astar and return the cost of the cheapest path for that node
	  nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
	  //    ros::Time t1 = ros::Time::now();
	  //    ros::Duration d(t1 - t0);
	  //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
	}
  
	if (Constants::twoD) {
	  // offset for same node in cell
	  twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
						((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
	  twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  
	}
  
	// return the maximum of the heuristics, making the heuristic admissable
	start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  }
package hybridastar

import (
	"math"
	"math/rand"
)

const node3dDir = 3

var (
	node3dDx = []float32{0.7068582, 0.705224, 0.705224}
	node3dDy = []float32{0, -0.0415893, 0.0415893}
	node3dDt = []float32{0, 0.1178097, -0.1178097}
)

type Node3D struct {
	// the x position
	x float32
	// the y position
	y float32
	// the heading theta
	t float32
	// the cost-so-far
	g float32
	// the cost-to-go
	h float32
	// the index of the node in the 3D array
	idx int32
	// the open value
	o bool
	// the closed value
	c bool
	// the motion primitive of the node
	prim int32
	// the predecessor pointer
	pred *Node3D
}

func NewNode3D(x, y, t, g, h float32, pred *Node3D, prim int32) *Node3D {
	return &Node3D{
		x:    x,
		y:    y,
		t:    t,
		g:    g,
		h:    h,
		pred: pred,
		o:    false,
		c:    false,
		idx:  -1,
		prim: prim,
	}
}

func (n *Node3D) GetX() float32 {
	return n.x
}

func (n *Node3D) GetY() float32 {
	return n.y
}

func (n *Node3D) GetT() float32 {
	return n.t
}

func (n *Node3D) GetG() float32 {
	return n.g
}

func (n *Node3D) GetH() float32 {
	return n.h
}

func (n *Node3D) GetC() float32 {
	return n.g + n.h
}

func (n *Node3D) GetIdx() int32 {
	return n.idx
}

func (n *Node3D) GetPrim() int32 {
	return n.prim
}

func (n *Node3D) IsOpen() bool {
	return n.o
}

func (n *Node3D) IsClosed() bool {
	return n.c
}

func (n *Node3D) GetPred() *Node3D {
	return n.pred
}

func (n *Node3D) SetX(x float32) {
	n.x = x
}

func (n *Node3D) SetY(y float32) {
	n.y = y
}

func (n *Node3D) SetT(t float32) {
	n.t = t
}

func (n *Node3D) SetG(g float32) {
	n.g = g
}

func (n *Node3D) SetH(h float32) {
	n.h = h
}

func (n *Node3D) SetIdx(idx int32) {
	n.idx = idx
}

func (n *Node3D) Open() {
	n.o = true
	n.c = false
}

func (n *Node3D) Close() {
	n.o = false
	n.c = true
}

func (n *Node3D) SetPred(pred *Node3D) {
	n.pred = pred
}

func (n *Node3D) UpdateG() {
	// forward driving
	if n.prim < 3 {
		// penalize turning
		if n.pred.prim != n.prim {
			// penalize change of direction
			if n.pred.prim > 2 {
				n.g += node3dDx[0] * penaltyTurning * penaltyCOD
			} else {
				n.g += node3dDx[0] * penaltyTurning
			}
		} else {
			n.g += node3dDx[0]
		}
	} else {
		// reverse driving

		// penalize turning and reversing
		if n.pred.prim != n.prim {
			// penalize change of direction
			if n.pred.prim < 3 {
				n.g += node3dDx[0] * penaltyTurning * penaltyReversing * penaltyCOD
			} else {
				n.g += node3dDx[0] * penaltyTurning * penaltyReversing
			}
		} else {
			n.g += node3dDx[0] * penaltyReversing
		}
	}
}

func (n *Node3D) Equal(rhs *Node3D) bool {
	return n.x == rhs.x &&
		n.y == rhs.y &&
		(float32(math.Abs(float64(n.t-rhs.t))) <= deltaHeadingRad ||
			float32(math.Abs(float64(n.t-rhs.t))) >= deltaHeadingNegRad)
}

func (n *Node3D) IsInRange(goal *Node3D) bool {
	random := rand.Int()%10 + 1
	dx := float32(math.Abs(float64(n.x-goal.x)) / float64(random))
	dy := float32(math.Abs(float64(n.y-goal.y)) / float64(random))
	return (dx*dx)+(dy*dy) < dubinsShotDistance
}

func (n *Node3D) IsOnGrid(width, height int32) bool {
	return n.x >= 0 && n.x < float32(width) &&
		n.y >= 0 && n.y < float32(height) &&
		n.t/deltaHeadingRad >= 0 && int32(n.t/deltaHeadingRad) < headings
}

func (n *Node3D) CreateSuccessor(i int32) *Node3D {
	var xSucc float32
	var ySucc float32
	var tSucc float32

	// calculate successor positions forward
	if i < 3 {
		xSucc = n.x + node3dDx[i]*float32(math.Cos(float64(n.t))) - node3dDy[i]*float32(math.Sin(float64(n.t)))
		ySucc = n.y + node3dDx[i]*float32(math.Sin(float64(n.t))) + node3dDy[i]*float32(math.Cos(float64(n.t)))
		tSucc = normalizeHeadingRad(n.t + node3dDt[i])
	} else {
		// backwards
		xSucc = n.x - node3dDx[i-3]*float32(math.Cos(float64(n.t))) - node3dDy[i-3]*float32(math.Sin(float64(n.t)))
		ySucc = n.y - node3dDx[i-3]*float32(math.Sin(float64(n.t))) + node3dDy[i-3]*float32(math.Cos(float64(n.t)))
		tSucc = normalizeHeadingRad(n.t - node3dDt[i-3])
	}

	return NewNode3D(xSucc, ySucc, tSucc, n.g, 0, n, i)
}

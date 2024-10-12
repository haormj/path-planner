package hybridastar

import "math"

// possible directions
const node2dDir int32 = 8

// possible movements
var (
	node2dDx = []int32{-1, -1, 0, 1, 1, 1, 0, -1}
	node2dDy = []int32{0, 1, 1, 1, 0, -1, -1, -1}
)

type Node2D struct {
	// the x position
	x int32
	// the y position
	y int32
	// the cost-so-far
	g float32
	// the cost-to-go
	h float32
	// the index of the node in the 2D array
	idx int32
	// the open value
	o bool
	// the closed value
	c bool
	// the discovered value
	d bool
	// the predecessor pointer
	pred *Node2D
}

func NewNode2D(x, y int32, g, h float32, pred *Node2D) *Node2D {
	return &Node2D{
		x:    x,
		y:    y,
		g:    g,
		h:    h,
		pred: pred,
		o:    false,
		c:    false,
		d:    false,
		idx:  -1,
	}
}

func (n *Node2D) GetX() int32 {
	return n.x
}

func (n *Node2D) GetY() int32 {
	return n.y
}

func (n *Node2D) GetT() float32 {
	// avoid 2D collision checking
	return 99
}

func (n *Node2D) GetG() float32 {
	return n.g
}

func (n *Node2D) GetH() float32 {
	return n.h
}

func (n *Node2D) GetC() float32 {
	return n.g + n.h
}

func (n *Node2D) GetIdx() int32 {
	return n.idx
}

func (n *Node2D) IsOpen() bool {
	return n.o
}

func (n *Node2D) IsClosed() bool {
	return n.c
}

func (n *Node2D) IsDiscovered() bool {
	return n.d
}

func (n *Node2D) GetPred() *Node2D {
	return n.pred
}

func (n *Node2D) SetX(x int32) {
	n.x = x
}

func (n *Node2D) SetY(y int32) {
	n.y = y
}

func (n *Node2D) SetG(g float32) {
	n.g = g
}

func (n *Node2D) SetH(h float32) {
	n.h = h
}

func (n *Node2D) SetIdx(width int32) int32 {
	n.idx = n.y*width + n.x
	return n.idx
}

func (n *Node2D) Open() {
	n.o = true
	n.c = false
}

func (n *Node2D) Close() {
	n.o = false
	n.c = true
}

func (n *Node2D) Reset() {
	n.o = false
	n.c = false
}

func (n *Node2D) Discover() {
	n.d = true
}

func (n *Node2D) SetPred(pred *Node2D) {
	n.pred = pred
}

func (n *Node2D) UpdateG() {
	n.g += n.MovementCost(n.pred)
	n.d = true
}

func (n *Node2D) UpdateH(goal *Node2D) {
	n.h = n.MovementCost(goal)
}

func (n *Node2D) MovementCost(pred *Node2D) float32 {
	return float32(math.Sqrt(float64((n.x-pred.x)*(n.x-pred.x) + (n.y-pred.y)*(n.y-pred.y))))
}

func (n *Node2D) Equal(rhs *Node2D) bool {
	return n.x == rhs.x && n.y == rhs.y
}

func (n *Node2D) IsOnGrid(width, height int32) bool {
	return n.x >= 0 && n.x < width && n.y >= 0 && n.y < height
}

func (n *Node2D) CreateSuccessor(i int32) *Node2D {
	xSucc := n.x + node2dDx[i]
	ySucc := n.y + node2dDy[i]
	return NewNode2D(xSucc, ySucc, n.g, 0, n)
}

// A Node2DMinHeap is a min-heap of Node2Ds.
type Node2DMinHeap []*Node2D

func (h Node2DMinHeap) Len() int           { return len(h) }
func (h Node2DMinHeap) Less(i, j int) bool { return h[i].GetC() < h[j].GetC() }
func (h Node2DMinHeap) Swap(i, j int)      { h[i], h[j] = h[j], h[i] }

func (h *Node2DMinHeap) Push(x interface{}) {
	// Push and Pop use pointer receivers because they modify the slice's length,
	// not just its contents.
	*h = append(*h, x.(*Node2D))
}

func (h *Node2DMinHeap) Pop() interface{} {
	old := *h
	n := len(old)
	x := old[n-1]
	*h = old[0 : n-1]
	return x
}

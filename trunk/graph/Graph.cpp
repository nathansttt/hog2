/*
 * $Id: graph.cpp,v 1.10 2006/10/24 18:23:20 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */ 

// HOG File

#include <cstdlib>
#include "Graph.h"

#include <vector>

using namespace std;

void graph_object::Print(ostream& /*out*/) const
{
	//	out << val;
}

ostream& operator <<(ostream & out, const graph_object &_Obj)
{
	_Obj.Print(out);
	return out;
}

graph::graph()
:_nodes(), _edges()
{
	//node_index = edge_index = 0;
}

graph::~graph()
{
  //	cout << "destructor got called" << endl;
  node_iterator ni;
  edge_iterator ei;
  ni = getNodeIter();
  while (1)
	{
    node *n = nodeIterNext(ni);
    if (n)
		{
      delete n;
      n = 0;
    }
    else
      break;
  }
	
  ei = getEdgeIter();
  while (1)
	{
    edge *e = edgeIterNext(ei);
    if (e)
		{
      delete e;
      e = 0;
    } else
      break;
  }
}

graph_object *graph::clone() const
{
  graph *g = new graph();
  node_iterator it = getNodeIter();
  while (1)
	{
    node *n = (node *)nodeIterNext(it);
    if (n) 
      g->AddNode((node*)n->clone());
    else
      break;
  }
  return g;
}

graph *graph::cloneAll() const
{
  graph *g = new graph();
  node_iterator it = getNodeIter();
  while (1)
	{
    node *n = (node *)nodeIterNext(it);
    if (n)
      g->AddNode((node*)n->clone());
    else
      break;
  }
  edge_iterator ei = getEdgeIter();
  while (1)
	{
    edge *e = (edge *)edgeIterNext(ei);
    if (e)
      g->AddEdge((edge*)e->clone());
    else
      break;
  }
  return g;
}

int graph::AddNode(node *n)
{
  if (n)
	{
    _nodes.push_back(n);
    n->nodeNum = _nodes.size()-1;
		//n->uniqueID = uniqueCounter++;
    //node_index;
    return n->nodeNum;
    //node_index++;
    //return node_index-1;
  }
  cerr << "ERRROR ERRROR ERRROR ERRROR ERRROR" << endl;
  return -1;
}

node *graph::getNode(unsigned int num)
{
  if (num >=0 && num < _nodes.size()) return _nodes[num];
  return 0;
}

void graph::AddEdge(edge *e)
{
  if (e)
	{
    _edges.push_back(e);
    e->edgeNum = _edges.size()-1;
    //_edges[edge_index] = e;
    if (e->getFrom() < _nodes.size())
      _nodes[e->getFrom()]->AddEdge(e);
    else
      cerr << "Adding edge from illegal index" << endl;
    if (e->getTo() < _nodes.size())
      _nodes[e->getTo()]->AddEdge(e);
    else
      cerr << "Adding edge to illegal index" << endl;
    //edge_index++;
  }
}

edge *graph::findDirectedEdge(unsigned int from, unsigned int to)
{
  node *n = getNode(from);
  edge_iterator ei = n->getOutgoingEdgeIter();
  while (1)
	{
    edge *e = n->edgeIterNextOutgoing(ei);
    if (e == 0)
      break;
    if (e->getTo() == to)
      return e;
  }
  return 0;
}

edge *graph::findEdge(unsigned int from, unsigned int to)
{
  node *n = getNode(from);
	if (n)
	{
		edge_iterator ei = n->getEdgeIter();
		while (1)
		{
			edge *e = n->edgeIterNext(ei);
			if (e == 0) break;
			if (((e->getTo() == to) && (e->getFrom() == from)) ||
					((e->getFrom() == to) && (e->getTo() == from)))
				return e;
		}
	}
  return 0;
}

bool graph::relax(edge *e, int weightIndex)
{
  int from = e->getFrom();
  int to = e->getTo();
  if (e == 0) return false;
  double newCost = getNode(from)->getLabelF(weightIndex)+e->getWeight();
  if (newCost <	getNode(to)->getLabelF(weightIndex))
	{
    //cout << "re-weighting" << endl << *getNode(to) << endl;
    getNode(to)->setLabelF(weightIndex, newCost);
    //cout << *getNode(to) << endl;
    return true;
  }
  return false;
}

bool graph::relaxReverseEdge(edge *e, int weightIndex)
{
  int from = e->getFrom();
  int to = e->getTo();
  if (e == 0) return false;
  double newCost = getNode(to)->getLabelF(weightIndex)+e->getWeight();
  if (newCost <	getNode(from)->getLabelF(weightIndex))
	{
    getNode(from)->setLabelF(weightIndex, newCost);
    return true;
  }
  return false;
}

node *graph::getRandomNode()
{
  if (_nodes.size() == 0) return 0;
  int rand_val = (int)(((double)random()/RAND_MAX)*_nodes.size());
  return _nodes[rand_val];
}

edge *graph::getRandomEdge()
{
  if (_edges.size() == 0) return 0;
  //	int rand_val = random()%edge_index;
  int rand_val = (int)(((double)random()/RAND_MAX)*_edges.size());
  return _edges[rand_val];
  //return 0;
}

// fixme: should be inlined
node_iterator graph::getNodeIter() const
{
  return _nodes.begin();
}

node *graph::nodeIterNext(node_iterator &node_iter) const
{
  if (node_iter != _nodes.end())
	{
    node *v = *node_iter;
    node_iter++;
    return v;
  }
  return 0;
}

// fixme: should be inlined
edge_iterator graph::getEdgeIter() const
{
  return _edges.begin();
}

edge *graph::edgeIterNext(edge_iterator &edge_iter) const
{
  if (edge_iter != _edges.end())
	{
    edge *v = *edge_iter;
    edge_iter++;
    return v;
  }
  return 0;
}

void graph::RemoveEdge(edge *e)
{
  //cout << "Removing edge " << e->edgeNum << " from " << e->from << " to " << e->to << endl;
  getNode(e->from)->RemoveEdge(e);
  getNode(e->to)->RemoveEdge(e);
  unsigned int oldLoc = e->edgeNum;
  edge *replacement = _edges.back();
  //cout << "Removing edge at " << oldLoc << " and putting " << replacement->edgeNum << " in its place" << endl;
  if (_edges[oldLoc] == e)
	{
    _edges[oldLoc] = replacement;
    replacement->edgeNum = oldLoc;
    _edges.pop_back();
  }
  else {
    cerr << "ERROR removing edge at " << oldLoc << endl;
    //		for (unsigned int x = 0; x < _edges.size(); x++)
    //		{
    //			if (_edges[x] == e)
    //				cout << "We found it at " << x << endl;
    //		}
  }
}

// returns the node that had it's node number changed, if any
node *graph::RemoveNode(node *n, unsigned int &oldID)
{
  if (!n) return 0;
	
  while (n->_allEdges.size() > 0)
	{
    edge *e = n->_allEdges.back();
    //		cout << "All edges contain " << n->_allEdges.size() << " edges." << endl;
    //		cout << "incoming has " << n->_edgesIncoming.size() << " and outgoing has "
    //			<< n->_edgesOutgoing.size() << " edges" << endl;
    //		cout << "Going to remove " << e << " " << *e << endl;
    if (e) RemoveEdge(e);
    else   break;
  }
  //printf("_nodes size is %u\n", _nodes.size());
  node *tmp = _nodes.back();
  _nodes.pop_back();
  if (_nodes.size() > 0)
	{
    _nodes[n->getNum()] = tmp;
    oldID = tmp->nodeNum;
    tmp->nodeNum = n->getNum();
  }
	
  if (n == tmp) return 0;
	
  // repair edges to and from this node...
  edge_iterator ei = tmp->getIncomingEdgeIter();
  for (edge *e = tmp->edgeIterNextIncoming(ei); e; e = tmp->edgeIterNextIncoming(ei))
	{
    e->to = tmp->nodeNum;
  }
  ei = tmp->getOutgoingEdgeIter();
  for (edge *e = tmp->edgeIterNextOutgoing(ei); e; e = tmp->edgeIterNextOutgoing(ei))
	{
    e->from = tmp->nodeNum;
  }
  return tmp;
}

// fixme: should be inlined
int graph::getNumEdges()
{
  return _edges.size();
}

// fixme: should be inlined
int graph::getNumNodes()
{
  return _nodes.size();
}

// BFS from start node
vector<node*>* graph::getReachableNodes(node* start)
{
  // Non-const array size trick in new compiler; appears as though
  // an initializer is not allowed
	std::vector<int> visitedList(_nodes.size());
  for (unsigned int i = 0; i < _nodes.size(); i++)
	{
    visitedList[i] = 0;
  }
  // Preallocation estimate
  vector<node*> *nodeList = new vector<node*>();
  nodeList->reserve(_nodes.size() * 3 / 4);
  unsigned int index = 0;
  int neighborNum = 0;
  node *n;
  
  nodeList->push_back(start);
  visitedList[start->getNum()] = 1;
  while (index < nodeList->size())
	{
    n = (*nodeList)[index];
    neighbor_iterator ni = n->getNeighborIter();
    while ((neighborNum = n->nodeNeighborNext(ni)) >= 0)
		{
      //cout << neighborNum << endl;
      if (visitedList[neighborNum] <= 0)
			{
        visitedList[neighborNum] = 1;
        nodeList->push_back(_nodes[neighborNum]);
      }
    }
    index++;
  }
  
  return nodeList;
}

void graph::Print(ostream &out) const
{
  node_iterator ni = getNodeIter();
  edge_iterator ei = getEdgeIter();
	
  out << "Nodes:" << endl;
  while (1)
	{
    node *n = nodeIterNext(ni);
    if (!n) break;
    out << *n << endl;
  }
  out << "Edges:" << endl;
  while (1)
	{
    edge *e = edgeIterNext(ei);
    if (!e) break;
    out << *e << endl;
  }
}

void graph::printStats()
{
	cout << getNumEdges() << " total edges; " << getNumNodes() << " total nodes." << endl;
	int minEdges = getNumEdges();
	int maxEdges[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  node_iterator ni = getNodeIter();
  while (1)
	{
    node *n = nodeIterNext(ni);
    if (!n) break;
		minEdges = min(minEdges, n->getNumEdges());
		for (int x = 0; x < 10; x++)
		{
			if (n->getNumEdges() > maxEdges[x])
			{
				for (int y = x; y < 9; y++)
					maxEdges[y+1] = maxEdges[y];
				maxEdges[x] = n->getNumEdges();
				break;
			}
		}
		//maxEdges = max(maxEdges, n->getNumEdges());
  }
	cout << "Min edges: " << minEdges << ", max edges: ";
	for (int x = 0; x < 10; x++)
		cout << maxEdges[x] << ", ";
	cout << endl;
}

bool graph::verifyGraph() const
{
  bool verified = true;
  int totalEdges1 = 0, totalEdges2 = 0;
  node_iterator ni = getNodeIter();
  edge_iterator ei;
	
  //	if (node_index != _nodes.size())
  //	{
  //		cerr << "Error: our node count doesn't match the size of nodes stored" << endl;
  //		verified = false;
  //	}
  //	if (edge_index != _edges.size())
  //	{
  //		cerr << "Error: our edge count doesn't match the size of edges stored" << endl;
  //		verified = false;
  //	}
	
  for (node *n = nodeIterNext(ni); n; n = nodeIterNext(ni))
	{
    if (!n)
		{
      cerr << "Error; we've stored null node" << endl;
      verified = false;
      continue;
    }
		else {
      //cout << "Testing node " << n->getNum() << endl;
    }
    if (_nodes[n->nodeNum] != n)
		{
      cerr << "Error; node " << n->nodeNum << "'s node number is off." << endl;
      verified = false;
    }
		
    ei = n->getEdgeIter();
    int supposedCount = n->getNumIncomingEdges()+n->getNumOutgoingEdges();
    int realCount = 0;
    for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
      totalEdges1++;
      realCount++;
      if ((e->getFrom() != n->getNum()) && (e->getTo() != n->getNum()))
			{
				cerr << "At node " << n->getNum() << " found edge between " << e->getFrom() << " and " << e->getTo() << endl;
				verified = false;
      }
    }
    if (realCount != supposedCount)
		{
      cerr << "At node " << n->getNum() << " supposed count is " << supposedCount << " but only found " << realCount << endl;
      verified = false;
    }
  }
	
  ei = getEdgeIter();
  for (edge *e = edgeIterNext(ei); e; e = edgeIterNext(ei))
	{
    if (_edges[e->edgeNum] != e)
		{
      cerr << "Error; edge " << e->edgeNum << "'s edge number is off." << endl;
      verified = false;
    }
		
    totalEdges2++;
  }
  if (totalEdges1/2 != totalEdges2)
	{
    cerr << "Edge counts don't match - from nodes " << totalEdges1 << ", from edges " << totalEdges2 << endl; 
    verified = false;
  }
  return verified;
}

edge::edge(unsigned int f, unsigned int t, double w)
: mark(false), from(f), to(t), label()
{ 
  setLabelF(kEdgeWeight, w);
	//	if ((from == -1) || (to == -1))
	//		cerr << "Error - " << from << "->" << to << endl;
}

void edge::Print(ostream& out) const
{
  out << from << "->" << to << " (" << getLabelF(kEdgeWeight) << ")";
}

void edge::setLabelF(unsigned int index, double val)
{
  if (index < label.size())
    label[index].fval = val;
  else {
    while (index > label.size())
		{
			labelValue v; v.fval = (double)MAXINT;
      label.push_back(v);
    }
		labelValue v; v.fval = val;
    label.push_back(v);
  }
}

void edge::setLabelL(unsigned int index, long val)
{
  if (index < label.size())
    label[index].lval = val;
  else {
    while (index > label.size())
		{
			labelValue v; v.lval = MAXINT;
      label.push_back(v);
    }
		labelValue v; v.lval = val;
    label.push_back(v);
  }
}


unsigned node::uniqueIDCounter = 0;

node::node(const char *n)
:label(), _edgesOutgoing(), _edgesIncoming(), _allEdges()
{
  // fixme: unsafe, 30?
  strncpy(name, n, 30);
  //	for (int x = 0; x < MAXLABELS; x++)
  //		label[x] = MAXINT;
  keyLabel = 0;
  markedEdge = 0;
	uniqueID = uniqueIDCounter++;
}

graph_object *node::clone() const
{
  node *n = new node(name);
  for (unsigned int x = 0; x < label.size(); x++) n->label.push_back(label[x]);
  n->keyLabel = keyLabel;
  return n;
}

void node::AddEdge(edge *e)
{
  if (e)
	{
    _allEdges.push_back(e);
    if (e->getFrom() == nodeNum)
      _edgesOutgoing.push_back(e);
    else if (e->getTo() == nodeNum)
      _edgesIncoming.push_back(e);
    else
      cerr << "Added an adge that doesn't belong to this node (" << nodeNum << ")" << endl;
  }
}

void node::RemoveEdge(edge *e)
{
  if (nodeNum == e->getTo())
	{
    for (unsigned int x = 0; x < _edgesIncoming.size(); x++)
		{
      if (_edgesIncoming[x] == e)
			{
				_edgesIncoming[x] = _edgesIncoming.back();
				_edgesIncoming.pop_back();
				break;
      }
    }
  }
	else {
    for (unsigned int x = 0; x < _edgesOutgoing.size(); x++)
		{
      if (_edgesOutgoing[x] == e)
			{
				_edgesOutgoing[x] = _edgesOutgoing.back();
				_edgesOutgoing.pop_back();
				break;
      }
    }
  }
  for (unsigned int x = 0; x < _allEdges.size(); x++)
	{
    if (_allEdges[x] == e)
		{
      _allEdges[x] = _allEdges.back();
      _allEdges.pop_back();
      break;
    }
  }
  if (markedEdge == e) markedEdge = 0;
}

void node::setLabelF(unsigned int index, double val)
{
  if (index < label.size())
    label[index].fval = val;
  else {
    while (index > label.size())
		{
			labelValue v; v.fval = (double)MAXINT;
      label.push_back(v);
    }
		labelValue v; v.fval = val;
    label.push_back(v);
  }
}

void node::setLabelL(unsigned int index, long val)
{
  if (index < label.size())
    label[index].lval = val;
  else {
    while (index > label.size())
		{
			labelValue v; v.lval = MAXINT;
      label.push_back(v);
    }
		labelValue v; v.lval = val;
    label.push_back(v);
  }
}

edge_iterator node::getIncomingEdgeIter() const
{
  return _edgesIncoming.begin();
}

edge *node::edgeIterNextIncoming(edge_iterator &iterIncoming) const
{
  if (iterIncoming != _edgesIncoming.end())
	{
    edge *v = *iterIncoming;
    iterIncoming++;
    return v;
  }
  return 0;
}

edge_iterator node::getOutgoingEdgeIter() const
{
  return _edgesOutgoing.begin();
}

edge *node::edgeIterNextOutgoing(edge_iterator &iterOutgoing) const
{
  if (iterOutgoing != _edgesOutgoing.end())
	{
    edge *v = *iterOutgoing;
    iterOutgoing++;
    return v;
  }
  return 0;
}

edge_iterator node::getEdgeIter() const
{
  return _allEdges.begin();
}

edge *node::edgeIterNext(edge_iterator &iter) const
{
  if (iter != _allEdges.end())
	{
    edge *v = *iter;
    iter++;
    return v;
  }
  return 0;
}

edge *node::getRandomIncomingEdge()
{
  int rand_val = random()%_edgesIncoming.size();
  return _edgesIncoming[rand_val];
}

edge *node::getRandomOutgoingEdge()
{
  int rand_val = random()%_edgesOutgoing.size();
  return _edgesOutgoing[rand_val];
}

edge *node::getRandomEdge()
{
  int rand_val = random()%_allEdges.size();
  return _allEdges[rand_val];
}

// fixme: should be inlined
int node::getNumOutgoingEdges()
{
  return _edgesOutgoing.size();
}

int node::getNumIncomingEdges()
{
  return _edgesIncoming.size();
}

edge *node::getEdge(unsigned int which)
{
	if (which > _allEdges.size())
		return 0;
	return _allEdges[which];
}


neighbor_iterator node::getNeighborIter() const
{
  return 0;
}

int node::nodeNeighborNext(neighbor_iterator& ni) const
{
  int result = -1;
  unsigned int os = _edgesOutgoing.size();
  if (ni < os)
    result = _edgesOutgoing[ni]->getTo();
  else if (ni - os < _edgesIncoming.size())
    result = _edgesIncoming[ni-os]->getFrom();
  ni++;
  return result;
}


void node::Print(ostream& out) const
{
  out << "\"" << name << "\"" << " (" << nodeNum << ")";
  for (unsigned int x = 0; x < label.size(); x++)
	{
    out << " - " << label[x].lval;
  }
}

ostream& operator <<(ostream & out, const graph &_Graph)
{
  _Graph.Print(out);
  return out;
}

ostream& operator <<(ostream & out, const node &_Node)
{
  _Node.Print(out);
  return out;
}

ostream& operator <<(ostream & out, const edge &_Edge)
{
  _Edge.Print(out);
  return out;
}

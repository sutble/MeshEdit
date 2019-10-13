#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
	//std::cout<<evaluatedLevels[0][0]<<'\n';
	//std::cout<<evaluatedLevels[0][1]<<'\n';
	//std::cout<<evaluatedLevels[0][2]<<'\n';
	//std::cout<<evaluatedLevels[0][3]<<'\n';

	std::vector<Vector2D> newControlPoints = {};
	int plvl = evaluatedLevels.size()-1;
	for(int i = 0; i < evaluatedLevels[plvl].size()-1; i ++){
		Vector2D newPoint = (1-t)*evaluatedLevels[plvl][i] + t*evaluatedLevels[plvl][i+1];
		newControlPoints.push_back(newPoint);
	}
	evaluatedLevels.push_back(newControlPoints);
    return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
	std::vector<Vector3D> final_points = {};
	for(int i = 0; i < 4; i ++){
		Vector3D point = evaluate1D(controlPoints[i], u);
		final_points.push_back(point);
	}
    return evaluate1D(final_points,v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

	  std::vector<Vector3D> intermediate_points = points;
	  for(int iter = 0; iter < 3; iter++){
		std::vector<Vector3D> new_points = {};
		for(int i = 0; i < intermediate_points.size()-1; i++){
		Vector3D newPoint = (1-t)*intermediate_points[i] + t*intermediate_points[i+1];
		new_points.push_back(newPoint);
		}
		intermediate_points = new_points;
	  }
		return intermediate_points[0];
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
	Vector3D n(0.,0.,0.);
	HalfedgeCIter h = halfedge();
	h = h->twin();
	HalfedgeCIter h_orig = h;
	do
	    {
	      Vector3D pi = h->vertex()->position;
	      Vector3D pj = h->next()->next()->vertex()->position;

	      n += cross(pj, pi);

	      h = h->next()->twin();
	    }
	    while( h != h_orig);

    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.

   //PHASE IA
   HalfedgeIter h0 = e0->halfedge();
   HalfedgeIter h1 = h0->next();
   HalfedgeIter h2 = h1->next();
   HalfedgeIter h3 = h0->twin();
   HalfedgeIter h4 = h3->next();
   HalfedgeIter h5 = h4->next();
   HalfedgeIter h6 = h1->twin();
   HalfedgeIter h7 = h2->twin();
   HalfedgeIter h8 = h4->twin();
   HalfedgeIter h9 = h5->twin();

   VertexIter v0 = h0->vertex();
   VertexIter v1 = h3->vertex();
   VertexIter v2 = h2->vertex();
   VertexIter v3 = h5->vertex();

   EdgeIter e1 = h1->edge();
   EdgeIter e2 = h2->edge();
   EdgeIter e3 = h4->edge();
   EdgeIter e4 = h5->edge();

   FaceIter f0 = h0->face();
   FaceIter f1 = h3->face();

   //CHECK BOUNDARY CONDITION ON THE 2 FACES
   if (f0->isBoundary() || f1->isBoundary()){
	   return e0;
   }

   //Phase IIA

   h0->next() = h1;
   h0->twin() = h3;
   h0->vertex() = v3;
   h0->edge() = e0;
   h0->face() = f0;

   h1->next() = h2;
   h1->twin() = h7;
   h1->vertex() = v2;
   h1->edge() = e2;
   h1->face() = f0;


   h2->next() = h0;
   h2->twin() = h8;
   h2->vertex() = v0;
   h2->edge() = e3;
   h2->face() = f0;

   h3->next() = h4;
   h3->twin() = h0;
   h3->vertex() = v2;
   h3->edge() = e0;
   h3->face() = f1;


   h4->next() = h5;
   h4->twin() = h9;
   h4->vertex() = v3;
   h4->edge() = e4;
   h4->face() = f1;

   h5->next() = h3;
   h5->twin() = h6;
   h5->vertex() = v1;
   h5->edge() = e1;
   h5->face() = f1;


   h6->next() = h6->next(); // didn’t change, but set it anyway!
   h6->twin() = h5;
   h6->vertex() = v2;
   h6->edge() = e1;
   h6->face() = h6->face(); // didn’t change, but set it an

   h7->next() = h7->next(); // didn’t change, but set it anyway!
   h7->twin() = h1;
   h7->vertex() = v0;
   h7->edge() = e2;
   h7->face() = h7->face(); // didn’t change, but set it an



   h8->next() = h8->next(); // didn’t change, but set it anyway!
   h8->twin() = h2;
   h8->vertex() = v3;
   h8->edge() = e3;
   h8->face() = h8->face(); // didn’t change, but set it an

   h9->next() = h9->next(); // didn’t change, but set it anyway!
   h9->twin() = h4;
   h9->vertex() = v1;
   h9->edge() = e4;
   h9->face() = h9->face(); // didn’t change, but set it an

   v0->halfedge() = h2;
   v1->halfedge() = h5;
   v2->halfedge() = h3;
   v3->halfedge() = h0;

   e0->halfedge() = h0;
   e1->halfedge() = h5;
   e2->halfedge() = h1;
   e3->halfedge() = h2;
   e4->halfedge() = h4;

   f0->halfedge() = h0;
   f1->halfedge() = h3;
   return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
	  //PHASE IA
	     HalfedgeIter h0 = e0->halfedge();
	     HalfedgeIter h1 = h0->next();
	     HalfedgeIter h2 = h1->next();
	     HalfedgeIter h3 = h0->twin();
	     HalfedgeIter h4 = h3->next();
	     HalfedgeIter h5 = h4->next();
	     HalfedgeIter h6 = h1->twin();
	     HalfedgeIter h7 = h2->twin();
	     HalfedgeIter h8 = h4->twin();
	     HalfedgeIter h9 = h5->twin();

	     VertexIter v0 = h0->vertex();
	     VertexIter v1 = h3->vertex();
	     VertexIter v2 = h2->vertex();
	     VertexIter v3 = h5->vertex();

	     EdgeIter e1 = h1->edge();
	     EdgeIter e2 = h2->edge();
	     EdgeIter e3 = h4->edge();
	     EdgeIter e4 = h5->edge();

	     FaceIter f0 = h0->face();
	     FaceIter f1 = h3->face();

	     //CHECK BOUNDARY CONDITION ON THE 2 FACES
	     if (f0->isBoundary() || f1->isBoundary()){
	  	   return v0;
	     }

	     //PHASE IB
	     VertexIter v4 = newVertex();
	     Vector3D p1 = v0->position;
	     Vector3D p2 = v1->position;
	     Vector3D midpoint = 0.5*(p1 + p2);
	     v4->position = midpoint;
	     FaceIter f2 = newFace();
	     FaceIter f3 = newFace();
	     EdgeIter e5 = newEdge();
	     EdgeIter e6 = newEdge();
	     EdgeIter e7 = newEdge();
	     HalfedgeIter h10 = newHalfedge();
	     HalfedgeIter h11 = newHalfedge();
	     HalfedgeIter h12 = newHalfedge();
	     HalfedgeIter h13 = newHalfedge();
	     HalfedgeIter h14 = newHalfedge();
	     HalfedgeIter h15 = newHalfedge();

	     //Phase IIA

		h0->next() = h1;
		h0->twin() = h3;
		h0->vertex() = v4;
		h0->edge() = e0;
		h0->face() = f0;

		h1->next() = h2;
		h1->twin() = h6;
		h1->vertex() = v1;
		h1->edge() = e1;
		h1->face() = f0;

		h2->next() = h0;
		h2->twin() = h10;
		h2->vertex() = v2;
		h2->edge() = e5;
		h2->face() = f0;

		h3->next() = h4;
		h3->twin() = h0;
		h3->vertex() = v1;
		h3->edge() = e0;
		h3->face() = f1;

		h4->next() = h5;
		h4->twin() = h15;
		h4->vertex() = v4;
		h4->edge() = e7;
		h4->face() = f1;

		h5->next() = h3;
		h5->twin() = h9;
		h5->vertex() = v3;
		h5->edge() = e4;
		h5->face() = f1;

		h6->next() = h6->next(); // didn’t change, but set it anyway!
		h6->twin() = h1;
		h6->vertex() = v2;
		h6->edge() = e1;
		h6->face() = h6->face(); // didn’t change, but set it an

		h7->next() = h7->next(); // didn’t change, but set it anyway!
		h7->twin() = h11;
		h7->vertex() = v0;
		h7->edge() = e2;
		h7->face() = h7->face(); // didn’t change, but set it an

		h8->next() = h8->next(); // didn’t change, but set it anyway!
		h8->twin() = h14;
		h8->vertex() = v3;
		h8->edge() = e3;
		h8->face() = h8->face(); // didn’t change, but set it an

		h9->next() = h9->next(); // didn’t change, but set it anyway!
		h9->twin() = h5;
		h9->vertex() = v1;
		h9->edge() = e4;
		h9->face() = h9->face(); // didn’t change, but set it an

		h10->next() = h11;
		h10->twin() = h2;
		h10->vertex() = v4;
		h10->edge() = e5;
		h10->face() = f3;

		h11->next() = h12;
		h11->twin() = h7;
		h11->vertex() = v2;
		h11->edge() = e2;
		h11->face() = f3;

		h12->next() = h10;
		h12->twin() = h13;
		h12->vertex() = v0;
		h12->edge() = e6;
		h12->face() = f3;

		h13->next() = h14;
		h13->twin() = h12;
		h13->vertex() = v4;
		h13->edge() = e6;
		h13->face() = f2;

		h14->next() = h15;
		h14->twin() = h8;
		h14->vertex() = v0;
		h14->edge() = e3;
		h14->face() = f2;

		h15->next() = h13;
		h15->twin() = h4;
		h15->vertex() = v3;
		h15->edge() = e7;
		h15->face() = f2;

		v0->halfedge() = h2;
		v1->halfedge() = h3;
		v2->halfedge() = h2;
		v3->halfedge() = h15;
		v4->halfedge() = h13;

		e0->halfedge() = h0;
		e1->halfedge() = h1;
		e2->halfedge() = h11;
		e3->halfedge() = h14;
		e4->halfedge() = h5;
		e5->halfedge() = h10;
		e6->halfedge() = h13;
		e7->halfedge() = h4;

		f0->halfedge() = h0;
		f1->halfedge() = h3;
		f2->halfedge() = h13;
		f3->halfedge() = h12;

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
	  std::vector<VertexIter> original_vertex_list;
	  for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++){
		  	 original_vertex_list.push_back(v);
	 		 v->isNew = false;
	 		 HalfedgeCIter h = v->halfedge();
	 		 Vector3D neighbor_position_sum(0.,0.,0.);
	 		 int n = 0;
	 	    do {
	 	        	HalfedgeCIter h_twin = h->twin(); // get the vertex of the current halfedge
	 	            neighbor_position_sum += h_twin->vertex()->position; // vertex is 'source' of the half edge.
	 	            n = n + 1;
	 	            h = h_twin->next();               // move to the next outgoing halfedge of the vertex.
	 	        } while(h != v->halfedge());
	 	    float u = 0;
	 	    if (n == 3){
	 	    	u = 0.1875;
	 	    }
	 	    else{
	 	    	u = 3/((float)8*n);
	 	    }
	 	    v->newPosition = (1-n*u)*v->position + u*neighbor_position_sum;
	 	 }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

	  std::vector<EdgeIter> original_edge_list;
	  for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
		  	  e->isNew = false;
		  	  original_edge_list.push_back(e);
	  	  }

	  for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
		  HalfedgeCIter h = e->halfedge();
		  Vector3D A = h->vertex()->position;
		  Vector3D B = h->twin()->vertex()->position;
		  Vector3D C = h->twin()->next()->twin()->vertex()->position;
		  Vector3D D = h->next()->twin()->vertex()->position;
		  e->newPosition = .375*(A+B) + .125*(C+D);
	  }


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)

	  for(int i=0; i < original_edge_list.size(); i++){
	     EdgeIter e = original_edge_list[i];
	     HalfedgeIter h = e->halfedge();
	     VertexIter v = mesh.splitEdge(e);
	     v->isNew = true;
	     v->position = e->newPosition;
	     v->halfedge()->edge()->isNew = false;
	     v->halfedge()->next()->next()->edge()->isNew = true;
	     v->halfedge()->twin()->next()->edge()->isNew = true;
	  }


    // TODO Now flip any new edge that connects an old and new vertex.

	  for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
		  	  if (!e->isNew){continue;}
	  		  HalfedgeIter h1 = e->halfedge();
	  		  HalfedgeIter h2 = h1->twin();
	  		  if (h1->vertex()->isNew && !h2->vertex()->isNew){
	  			  mesh.flipEdge(e);
	  		  }
	  		  if (!h1->vertex()->isNew && h2->vertex()->isNew){
	  			  			  mesh.flipEdge(e);
	  		  }
	  	  }


    // TODO Finally, copy the new vertex positions into final Vertex::position.


	  for(int i=0; i < original_vertex_list.size(); i++){
	     VertexIter v = original_vertex_list[i];
	     v->position = v->newPosition;
	  }

    return;
  }
}

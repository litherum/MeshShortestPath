#include "stdafx.h"

#include "GenerateGeometry.h"
#include <algorithm>

namespace MeshShortestPath {

	class Generator : public CGAL::Modifier_base<HalfedgeDS> {
	private:
		Polyhedron::Facet_handle addTriangle(CGAL::Polyhedron_incremental_builder_3<HalfedgeDS>& builder, std::size_t index0, std::size_t index1, std::size_t index2) {
			auto facet = builder.begin_facet();
			builder.add_vertex_to_facet(index0);
			builder.add_vertex_to_facet(index1);
			builder.add_vertex_to_facet(index2);
			builder.end_facet();
			return facet;
		}

		void operator()(HalfedgeDS& halfedgeDS) {
			CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder(halfedgeDS, true);
			builder.begin_surface(0, 0, 0);
			builder.add_vertex(Polyhedron::Point(0, 0, 1));
			builder.add_vertex(Polyhedron::Point(0, 1, 1));
			builder.add_vertex(Polyhedron::Point(1, 1, 1));
			builder.add_vertex(Polyhedron::Point(1, 0, 1));
			builder.add_vertex(Polyhedron::Point(0, 0, 0));
			builder.add_vertex(Polyhedron::Point(0, 1, 0));
			builder.add_vertex(Polyhedron::Point(1, 1, 0));
			builder.add_vertex(Polyhedron::Point(1, 0, 0));
			// Front
			addTriangle(builder, 0, 1, 2);
			addTriangle(builder, 2, 3, 0);
			// Left
			addTriangle(builder, 4, 5, 1);
			addTriangle(builder, 1, 0, 4);
			// Right
			addTriangle(builder, 3, 2, 6);
			addTriangle(builder, 6, 7, 3);
			// Back
			addTriangle(builder, 7, 6, 5);
			addTriangle(builder, 5, 4, 7);
			// Top
			addTriangle(builder, 1, 5, 6);
			addTriangle(builder, 6, 2, 1);
			// Bottom
			addTriangle(builder, 4, 0, 3);
			addTriangle(builder, 3, 7, 4);
			builder.end_surface();
		}
	};

	class PlaneEquation {
	public:
		Polyhedron::Plane_3 operator()(Polyhedron::Facet& facet) {
			auto halfedge = facet.halfedge();
			auto p0 = halfedge->vertex()->point();
			auto p1 = halfedge->next()->vertex()->point();
			auto p2 = halfedge->next()->next()->vertex()->point();
			return Polyhedron::Plane_3(p0, p1, p2);
		}
	};

	Polyhedron generateCube() {
		Polyhedron polyhedron;
		Generator generator;
		polyhedron.delegate(generator);
		std::transform(polyhedron.facets_begin(), polyhedron.facets_end(), polyhedron.planes_begin(), PlaneEquation());
		return polyhedron;
	}
}
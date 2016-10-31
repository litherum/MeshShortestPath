#include "stdafx.h"

#include "GenerateGeometry.h"

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
			builder.add_vertex(Polyhedron::Point(0, 0, 0));
			builder.add_vertex(Polyhedron::Point(0, 1, 0));
			builder.add_vertex(Polyhedron::Point(1, 1, 0));
			builder.add_vertex(Polyhedron::Point(1, 0, 0));
			builder.add_vertex(Polyhedron::Point(0, 0, 1));
			builder.add_vertex(Polyhedron::Point(0, 1, 1));
			builder.add_vertex(Polyhedron::Point(1, 1, 1));
			builder.add_vertex(Polyhedron::Point(1, 0, 1));
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

	Polyhedron generateCube() {
		Polyhedron polyhedron;
		Generator generator;
		polyhedron.delegate(generator);
		return polyhedron;
	}
}
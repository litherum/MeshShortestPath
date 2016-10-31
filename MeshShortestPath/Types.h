#pragma once

#include <list>

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

namespace MeshShortestPath {

	class CandidateInterval {

	};

	typedef CGAL::Simple_cartesian<double> Kernel;

	template <class Refs>
	class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
	public:

	private:
		std::list<std::list<CandidateInterval>::iterator> intervals;
	};

	struct MMP_Items : public CGAL::Polyhedron_items_3 {
		template <class Refs, class Traits>
		struct Halfedge_wrapper {
			typedef HalfedgeWithIntervalVector<Refs> Halfedge;
		};
	};

	typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;
	typedef Polyhedron::HalfedgeDS HalfedgeDS;

}

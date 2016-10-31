#pragma once

#include <list>
#include <vector>

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

#include <boost/variant.hpp>

namespace MeshShortestPath {

	typedef CGAL::Simple_cartesian<double> Kernel;

	class CandidateInterval;

	void insertInterval(std::list<CandidateInterval>::iterator interval, std::vector<std::list<CandidateInterval>::iterator>& intervals);

	template <class Refs>
	class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
	public:
		void insertInterval(std::list<CandidateInterval>::iterator interval) {
			MeshShortestPath::insertInterval(interval, intervals);
		}

	private:
		std::vector<std::list<CandidateInterval>::iterator> intervals;
	};

	struct MMP_Items : public CGAL::Polyhedron_items_3 {
		template <class Refs, class Traits>
		struct Halfedge_wrapper {
			typedef HalfedgeWithIntervalVector<Refs> Halfedge;
		};
	};

	typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;
	typedef Polyhedron::HalfedgeDS HalfedgeDS;

	class CandidateInterval {
	public:
	CandidateInterval(
		Polyhedron::Halfedge_const_handle halfedge,
		Kernel::Point_3 root,
		Kernel::Point_3 unfoldedRoot,
		boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor,
		Kernel::FT depth,
		Kernel::FT leftExtent,
		Kernel::FT rightExtent) :
			halfedge(halfedge),
			root(root),
			unfoldedRoot(unfoldedRoot),
			predecessor(predecessor),
			depth(depth),
			leftExtent(leftExtent),
			rightExtent(rightExtent) {
	}

	private:
	Polyhedron::Halfedge_const_handle halfedge;
	Kernel::Point_3 root;
	Kernel::Point_3 unfoldedRoot;
	Kernel::Point_3 frontierPoint;
	Kernel::Point_3 accessPoint;
	boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor;
	Kernel::FT depth;
	Kernel::FT leftExtent;
	Kernel::FT rightExtent;
	};

}

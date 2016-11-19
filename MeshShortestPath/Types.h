#pragma once

#include <list>
#include <vector>
#include <memory>

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

#include <boost/variant.hpp>

namespace MeshShortestPath {

	typedef CGAL::Simple_cartesian<double> Kernel;

	class CandidateInterval;

	bool insertInterval(std::list<CandidateInterval>::iterator interval, std::vector<std::list<CandidateInterval>::iterator>& intervals);

	template <class Refs>
	class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
	public:
		bool insertInterval(std::list<CandidateInterval>::iterator interval) {
			return MeshShortestPath::insertInterval(interval, intervals);
		}

		void clear() {
			intervals.clear();
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
			Polyhedron::Halfedge_handle halfedge,
			//Kernel::Point_3 root,
			Kernel::Point_3 unfoldedRoot,
			//boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor,
			Kernel::FT depth,
			Kernel::FT lowerExtent,
			Kernel::FT upperExtent);

		Polyhedron::Halfedge_handle getHalfedge() const { return halfedge; }
		Kernel::Point_3 getUnfoldedRoot() const { return unfoldedRoot; }
		Kernel::FT getDepth() const { return depth; }
		Kernel::Point_3 getFrontierPoint() const { return frontierPoint; }
		Kernel::Point_3 getLowerExtent() const;
		Kernel::Point_3 getUpperExtent() const;
		bool getFrontierPointIsAtExtent() const { return frontierPointIsAtExtent; }
		void setFrontierPointIsAtVertex(bool isAtVertex) { frontierPointIsAtVertex = isAtVertex; }
		bool getFrontierPointIsAtVertex() const { return frontierPointIsAtVertex; }

	private:
		Polyhedron::Halfedge_handle halfedge;
		//Kernel::Point_3 root;
		Kernel::Point_3 unfoldedRoot;
		Kernel::Point_3 frontierPoint;
		Kernel::Point_3 accessPoint;
		//boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor;
		Kernel::FT depth;
		Kernel::FT lowerExtent;
		Kernel::FT upperExtent;
		bool frontierPointIsAtExtent { false };
		bool frontierPointIsAtVertex { false };
	};

	class FrontierPointEvent {
	public:
		FrontierPointEvent(Kernel::Point_3 point, std::list<CandidateInterval>::iterator candidateInterval) : point(point), candidateInterval(candidateInterval) {
		}

		std::list<CandidateInterval>::iterator getCandidateInterval() const {
			return candidateInterval;
		}

		Kernel::FT getLabel() const;

	private:
		Kernel::Point_3 point;
		std::list<CandidateInterval>::iterator candidateInterval;
	};

	class EndPointEvent {
	public:
		EndPointEvent(Kernel::Point_3 point, const CandidateInterval& candidateInterval);

		Kernel::FT getLabel() const {
			return label;
		}

	private:
		Kernel::Point_3 point;
		Kernel::FT label;
	};

	typedef boost::variant<FrontierPointEvent, EndPointEvent> GenericEvent;

	class EventQueue {
	public:
		void place(FrontierPointEvent&);
		void place(EndPointEvent&);
		GenericEvent remove();
		bool empty() const {
			return heap.empty();
		}

	private:
		std::vector<GenericEvent> heap;
	};
}

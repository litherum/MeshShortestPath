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
			Kernel::FT rightExtent);

		Kernel::Point_3 getUnfoldedRoot() const { return unfoldedRoot; }
		Kernel::FT getDepth() const { return depth; }
		Kernel::Point_3 getFrontierPoint() const { return frontierPoint; }
		Kernel::Point_3 getLeftExtent() const;
		Kernel::Point_3 getRightExtent() const;

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

	// FIXME: Use variants instead to avoid allocations
	class Event {
	public:
		virtual Kernel::FT getLabel() const = 0;
	};

	class FrontierPointEvent : public Event {
	public:
		FrontierPointEvent(Kernel::Point_3 point, std::list<CandidateInterval>::iterator candidateInterval) : point(point), candidateInterval(candidateInterval) {
		}

	private:
		Kernel::FT getLabel() const override;

		Kernel::Point_3 point;
		std::list<CandidateInterval>::iterator candidateInterval;
	};

	class EndPointEvent : public Event {
	public:
		EndPointEvent(Kernel::Point_3 point, const CandidateInterval& candidateInterval);

	private:
		Kernel::FT getLabel() const override {
			return label;
		}

		Kernel::Point_3 point;
		Kernel::FT label;
	};

	class EventQueue {
	public:
		void place(std::unique_ptr<Event>&&);
		std::unique_ptr<Event> remove();
		bool empty() const {
			return heap.empty();
		}

	private:
		std::vector<std::unique_ptr<Event>> heap;
	};
}

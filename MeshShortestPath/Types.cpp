#include "stdafx.h"

#include "Types.h"

#include <algorithm>

namespace MeshShortestPath {

	bool insertInterval(std::list<CandidateInterval>::iterator interval, std::vector<std::list<CandidateInterval>::iterator>& intervals) {
		// FIXME: Implement this
		intervals.push_back(interval);
		return true;
	}

	CandidateInterval::CandidateInterval(
		Polyhedron::Halfedge_handle halfedge,
		//Kernel::Point_3 root,
		Kernel::Point_3 unfoldedRoot,
		//boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor,
		Kernel::FT depth,
		Kernel::FT lowerExtent,
		Kernel::FT upperExtent) :
			halfedge(halfedge),
			//root(root),
			unfoldedRoot(unfoldedRoot),
			//predecessor(predecessor),
			depth(depth),
			lowerExtent(lowerExtent),
			upperExtent(upperExtent) {
		assert(lowerExtent <= upperExtent);
		assert(lowerExtent >= 0);
		assert(upperExtent <= 1);
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		// Project unfoldedRoot onto the line from source to destination
		Kernel::Vector_3 a(unfoldedRoot.x() - source.x(), unfoldedRoot.y() - source.y(), unfoldedRoot.z() - source.z());
		Kernel::Vector_3 b(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		auto projectionFraction = (a * b) / (b * b);

		// FIXME: Provide some more resilient way to test for frontier points coincident with vertices
		if (projectionFraction <= lowerExtent) {
			projectionFraction = lowerExtent;
			frontierPointIsAtExtent = true;
		}
		else if (projectionFraction >= upperExtent) {
			projectionFraction = upperExtent;
			frontierPointIsAtExtent = true;
		}
		frontierPoint = source + projectionFraction * b;
		// FIXME: calculate accessPoint by intersecting a line between frontierPoint and unfoldedRoot with the other edges of the facet.
	}

	Kernel::Point_3 CandidateInterval::getLowerExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		return source + lowerExtent * s;
	}

	Kernel::Point_3 CandidateInterval::getUpperExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		return source + upperExtent * s;
	}

	// Precondition: Point is part of the candidate interval.
	static Kernel::FT computeLabel(Kernel::Point_3 point, const CandidateInterval& candidateInterval) {
		auto unfoldedRoot = candidateInterval.getUnfoldedRoot();
		Kernel::Vector_3 displacement(point.x() - unfoldedRoot.x(), point.y() - unfoldedRoot.y(), point.z() - unfoldedRoot.z());
		return std::sqrt(displacement.squared_length()) + candidateInterval.getDepth();
	}

	Kernel::FT FrontierPointEvent::getLabel() const {
		return computeLabel(point, *candidateInterval);
	}

	EndPointEvent::EndPointEvent(Kernel::Point_3 point, const CandidateInterval& candidateInterval) : label(computeLabel(point, candidateInterval)) {
	}

	class EventComparison {
	public:
		Kernel::FT operator()(const FrontierPointEvent& event) const {
			return event.getLabel();
		}

		Kernel::FT operator()(const EndPointEvent& event) const {
			return event.getLabel();
		}
	};

	static bool eventComparison(const GenericEvent& a, const GenericEvent& b) {
		Kernel::FT aLabel = boost::apply_visitor(EventComparison(), a);
		Kernel::FT bLabel = boost::apply_visitor(EventComparison(), b);
		return aLabel > bLabel;
	}

	void EventQueue::place(FrontierPointEvent& event) {
		heap.push_back(event);
		std::push_heap(heap.begin(), heap.end(), &eventComparison);
	}

	void EventQueue::place(EndPointEvent& event) {
		heap.push_back(event);
		std::push_heap(heap.begin(), heap.end(), &eventComparison);
	}

	GenericEvent EventQueue::remove() {
		std::pop_heap(heap.begin(), heap.end(), &eventComparison);
		GenericEvent result = heap.back();
		heap.pop_back();
		return result;
	}
}
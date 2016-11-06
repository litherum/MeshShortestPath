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
		Kernel::Point_3 root,
		Kernel::Point_3 unfoldedRoot,
		//boost::variant<Kernel::Point_3, std::reference_wrapper<CandidateInterval>> predecessor,
		Kernel::FT depth,
		Kernel::FT leftExtent,
		Kernel::FT rightExtent) :
			halfedge(halfedge),
			root(root),
			unfoldedRoot(unfoldedRoot),
			//predecessor(predecessor),
			depth(depth),
			leftExtent(leftExtent),
			rightExtent(rightExtent) {
		assert(leftExtent <= rightExtent);
		assert(leftExtent >= 0);
		assert(rightExtent <= 1);
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		// Project unfoldedRoot onto the line from source to destination
		Kernel::Vector_3 a(unfoldedRoot.x() - source.x(), unfoldedRoot.y() - source.y(), unfoldedRoot.z() - source.z());
		Kernel::Vector_3 b(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		auto projectionFraction = (a * b) / (b * b);

		// FIXME: Provide some more resilient way to test for frontier points coincident with vertices
		if (projectionFraction <= leftExtent) {
			projectionFraction = leftExtent;
			frontierPointIsAtExtent = true;
		}
		else if (projectionFraction >= rightExtent) {
			projectionFraction = rightExtent;
			frontierPointIsAtExtent = true;
		}
		frontierPoint = source + projectionFraction * b;
		// FIXME: calculate accessPoint by intersecting a line between frontierPoint and unfoldedRoot with the other edges of the facet.
	}

	Kernel::Point_3 CandidateInterval::getLeftExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		return source + leftExtent * s;
	}

	Kernel::Point_3 CandidateInterval::getRightExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		return source + rightExtent * s;
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

	static bool eventComparison(const std::unique_ptr<Event>& a, const std::unique_ptr<Event>& b) {
		return a->getLabel() > b->getLabel();
	}

	void EventQueue::place(std::unique_ptr<Event>&& event) {
		heap.emplace_back(std::move(event));
		std::push_heap(heap.begin(), heap.end(), &eventComparison);
	}

	std::unique_ptr<Event> EventQueue::remove() {
		std::pop_heap(heap.begin(), heap.end(), &eventComparison);
		std::unique_ptr<Event> result = std::move(heap.back());
		assert(heap.back() == nullptr);
		heap.pop_back();
		return result;
	}
}
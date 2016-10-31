#include "stdafx.h"

#include "Types.h"

#include <algorithm>

namespace MeshShortestPath {

	void insertInterval(std::list<CandidateInterval>::iterator interval, std::vector<std::list<CandidateInterval>::iterator>& intervals) {
		// FIXME: Implement this
		intervals.push_back(interval);
	}

	CandidateInterval::CandidateInterval(
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
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		// Project unfoldedRoot onto the line from source to destination
		Kernel::Vector_3 v(unfoldedRoot.x() - source.x(), unfoldedRoot.y() - source.y(), unfoldedRoot.z() - source.z());
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		auto projectionFraction = (v * s) / (s * s);
		bool frontierPointIsLeftExtent = false;
		bool frontierPointIsRightExtent = false;
		if (projectionFraction <= leftExtent) {
			projectionFraction = leftExtent;
			frontierPointIsLeftExtent = true;
		}
		else if (projectionFraction >= rightExtent) {
			projectionFraction = rightExtent;
			frontierPointIsRightExtent = true;
		}
		frontierPoint = source + projectionFraction * s;
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

	bool Event::operator<(const Event& other) const {
		auto computeDistance = [](const Event& event) {
			auto point = event.point;
			auto unfoldedRoot = event.candidateInterval->getUnfoldedRoot();
			Kernel::Vector_3 displacement(point.x() - unfoldedRoot.x(), point.y() - unfoldedRoot.y(), point.z() - unfoldedRoot.z());
			return std::sqrt(displacement.squared_length()) + event.candidateInterval->getDepth();
		};
		auto distance0 = computeDistance(*this);
		auto distance1 = computeDistance(other);
		return distance0 < distance1;
	}

	void EventQueue::place(Event event) {
		heap.push_back(event);
		std::push_heap(heap.begin(), heap.end());
	}

	Event EventQueue::remove() {
		std::pop_heap(heap.begin(), heap.end());
		Event result = heap.back();
		heap.pop_back();
		return result;
	}

	bool EventQueue::empty() const {
		return heap.empty();
	}
}
#include "stdafx.h"

#include "Types.h"

#include <algorithm>
#include <iterator>
#include <cassert>

namespace MeshShortestPath {

	static Kernel::FT distanceBetweenPoints(Polyhedron::Point_3 a, Polyhedron::Point_3 b) {
		return std::sqrt(Kernel::Vector_3(b.x() - a.x(), b.y() - a.y(), b.z() - a.z()).squared_length());
	}

	template <typename T>
	static T findEndOfDominatedIntervals(T begin, T end, Polyhedron::Point_3 (CandidateInterval::*extentFunction)() const, const CandidateInterval& interval) {
		while (begin != end) {
			if (distanceBetweenPoints(((**begin).*extentFunction)(), (*begin)->getUnfoldedRoot()) + (*begin)->getDepth() >=
				distanceBetweenPoints(((**begin).*extentFunction)(), interval.getUnfoldedRoot()) + interval.getDepth())
				++begin;
			else
				break;
		}
		return begin;
	}

	boost::optional<bool> insertInterval(std::list<CandidateInterval>::iterator interval, std::vector<std::list<CandidateInterval>::iterator>& intervals, const CandidateInterval& predecessor, std::function<std::list<CandidateInterval>::iterator(CandidateInterval)> addCandidateInterval) {
		CandidateInterval::AccessPoint searchFor = { 1 - predecessor.frontierPoint, predecessor.getHalfedge()->opposite() == interval->getHalfedge()->next() };

		auto searchComparison = [](const CandidateInterval::AccessPoint probe, const std::list<CandidateInterval>::iterator& existing) {
			if (existing->accessPoint.initialSide && !probe.initialSide)
				return true;
			if (!existing->accessPoint.initialSide && probe.initialSide)
				return false;
			return existing->accessPoint.location < probe.location;
		};

		auto location = std::upper_bound(intervals.begin(), intervals.end(), searchFor, searchComparison);

		auto beginDeleting = findEndOfDominatedIntervals(std::make_reverse_iterator(location), intervals.rend(), &CandidateInterval::getLowerExtent, *interval);
		auto endDeleting = findEndOfDominatedIntervals(location, intervals.end(), &CandidateInterval::getUpperExtent, *interval);

		if (beginDeleting != intervals.rend()) {
			// FIXME: Trim *beginDeleting and interval.
			// Modifying the extents may require recomputation of the frontier point and access point.
		}

		if (endDeleting != intervals.end()) {
			// FIXME: Trim *endDeleting and interval.
			// Modifying the extents may require recomputation of the frontier point and access point.
		}

		// FIXME: Figure out when we should be returning boost::none.

		// The event queue has iterators into the candidateIntervals list, so we can't simply delete them without updating the event queue.
		std::for_each(beginDeleting.base(), endDeleting, [](auto candidateInterval) {
			candidateInterval->setDeleted();
		});
		bool result = beginDeleting == intervals.rend() || endDeleting == intervals.end();

		auto toInsert = intervals.erase(beginDeleting.base(), endDeleting);
		intervals.insert(toInsert, interval);

		return result;
	}

	CandidateInterval::CandidateInterval(
		Polyhedron::Halfedge_handle halfedge,
		Kernel::Point_3 unfoldedRoot,
		Kernel::FT depth,
		Kernel::FT lowerExtent,
		Kernel::FT upperExtent) :
			halfedge(halfedge),
			unfoldedRoot(unfoldedRoot),
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
		frontierPoint = (a * b) / (b * b);

		// FIXME: Provide some more resilient way to test for frontier points coincident with vertices
		if (frontierPoint <= lowerExtent) {
			frontierPoint = lowerExtent;
			frontierPointIsAtExtent = true;
		}
		else if (frontierPoint >= upperExtent) {
			frontierPoint = upperExtent;
			frontierPointIsAtExtent = true;
		}

		accessPoint = calculateAccessPoint();
	}

	auto CandidateInterval::calculateAccessPoint() const -> AccessPoint {
		auto initialSideFraction = lineLineIntersection(halfedge->vertex()->point(), halfedge->next()->vertex()->point(), unfoldedRoot, getFrontierPoint());
		if (initialSideFraction > 1 || initialSideFraction < 0) {
			auto secondarySideFraction = lineLineIntersection(halfedge->next()->vertex()->point(), halfedge->next()->next()->vertex()->point(), unfoldedRoot, getFrontierPoint());
			if (secondarySideFraction > 1 || secondarySideFraction < 0) {
				auto initialSideDistance = std::min(std::abs(initialSideFraction), std::abs(initialSideFraction - 1));
				auto secondarySideDistance = std::min(std::abs(secondarySideFraction), std::abs(secondarySideFraction - 1));
				if (initialSideDistance < secondarySideDistance)
					return { std::min(std::max(initialSideFraction, Kernel::FT(0)), Kernel::FT(1)), true };
				else
					return { std::min(std::max(secondarySideFraction, Kernel::FT(0)), Kernel::FT(1)), false };
			}
			else
				return { secondarySideFraction, false };
		}
		else
			return { initialSideFraction, true };
	}

	Kernel::Point_3 CandidateInterval::getFrontierPoint() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		Kernel::Vector_3 s(destination.x() - source.x(), destination.y() - source.y(), destination.z() - source.z());
		return source + frontierPoint * s;
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

	static bool checkLineLineIntersectionResult(Kernel::FT result, Polyhedron::Point_3 p1, Kernel::Vector_3 v1, Polyhedron::Point_3 p2, Kernel::Vector_3 v2) {
		auto resultPoint = p1 + result * v1;
		Kernel::Line_3 l2(p2, v2);
		auto projected = l2.projection(resultPoint);
		auto displacement = projected - resultPoint;
		return displacement.squared_length() <= 0.001;
	}

	Kernel::FT lineLineIntersection(Polyhedron::Point_3 a1, Polyhedron::Point_3 a2, Polyhedron::Point_3 b1, Polyhedron::Point_3 b2) {
		// Calculates the fraction of the distance between a1 and a2 the intersection occurs.
		Kernel::Vector_3 av(a2.x() - a1.x(), a2.y() - a1.y(), a2.z() - a1.z());
		Kernel::Vector_3 bv(b2.x() - b1.x(), b2.y() - b1.y(), b2.z() - b1.z());

		// a + t*b = c + s*d, f + t*g = h + s*j
		// t = (-a * j + c * j + d * f - d * h) / (b * j - d * g)

		auto calculateDenominator = [](Kernel::FT v1p, Kernel::FT v1q, Kernel::FT v2p, Kernel::FT v2q) {
			// b, g, d, j
			// return b * j - d * g
			return v1p * v2q - v2p * v1q;
		};

		auto calculateNumerator = [](Kernel::FT p1p, Kernel::FT p1q, Kernel::FT p2p, Kernel::FT p2q, Kernel::FT v2p, Kernel::FT v2q) {
			// a, f, c, h, d, j
			// return -a * j + c * j + d * f - d * h
			return -p1p * v2q + p2p * v2q + v2p * p1q - v2p * p2q;
		};

		auto denominatorXY = calculateDenominator(av.x(), av.y(), bv.x(), bv.y());
		auto denominatorXZ = calculateDenominator(av.x(), av.z(), bv.x(), bv.z());
		auto denominatorYZ = calculateDenominator(av.y(), av.z(), bv.y(), bv.z());

		auto denominatorXYAbs = std::abs(denominatorXY);
		auto denominatorXZAbs = std::abs(denominatorXZ);
		auto denominatorYZAbs = std::abs(denominatorYZ);

		Kernel::FT t;

		if (denominatorXYAbs >= denominatorXZAbs && denominatorXYAbs >= denominatorYZAbs) {
			// Use XY
			auto numerator = calculateNumerator(a1.x(), a1.y(), b1.x(), b1.y(), bv.x(), bv.y());
			t = numerator / denominatorXY;
		}
		else if (denominatorXZAbs >= denominatorXYAbs && denominatorXZAbs >= denominatorYZAbs) {
			// Use XZ
			auto numerator = calculateNumerator(a1.x(), a1.z(), b1.x(), b1.z(), bv.x(), bv.z());
			t = numerator / denominatorXZ;
		}
		else {
			// Use YZ
			auto numerator = calculateNumerator(a1.y(), a1.z(), b1.y(), b1.z(), bv.y(), bv.z());
			t = numerator / denominatorYZ;
		}
		assert(checkLineLineIntersectionResult(t, a1, av, b1, bv));
		return t;
	}
}
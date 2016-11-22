#include "stdafx.h"

#include "Types.h"

#include <algorithm>
#include <iterator>
#include <cassert>

namespace MeshShortestPath {

	void printPoint(std::ostream& stream, Polyhedron::Point point) {
		stream << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
	}

	static Kernel::FT distanceBetweenPoints(Polyhedron::Point_3 a, Polyhedron::Point_3 b) {
		return std::sqrt(Kernel::Vector_3(b.x() - a.x(), b.y() - a.y(), b.z() - a.z()).squared_length());
	}

	static Kernel::FT projectionScalar(Polyhedron::Halfedge_handle halfedge, Polyhedron::Point_3 point) {
		auto source = halfedge->opposite()->vertex()->point();
		auto destination = halfedge->vertex()->point();
		auto b = destination - source;
		auto a = point - source;
		return (a * b) / (b * b);
	}

	static std::vector<Kernel::FT> quadraticFormula(Kernel::FT a, Kernel::FT b, Kernel::FT c) {
		auto discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
			return {};
		else if (discriminant == 0) // FIXME: Possibly round a little
			return { -b / (2 * a) };
		else {
			auto discriminantSquareRoot = std::sqrt(discriminant);
			return { (-b + discriminantSquareRoot) / (2 * a), (-b - discriminantSquareRoot) / (2 * a) };
		}
	}

	static std::vector<Kernel::FT> calculateTiePoints(Kernel::FT d1, Kernel::FT r1, Kernel::FT b, Kernel::FT d2, Kernel::FT r2) {
		// b = d1 + d2
		// d1 + sqrt(r1^2 + b1^2) = d2 + sqrt(r2^2 + b2^2)
		auto d = d1 - d2;
		auto ra = r1 * r1;
		auto rb = r2 * r2;
		// d + sqrt(ra + b1^2) = sqrt(rb + (b - b1)^2)

		if (d == 0) {
			// sqrt(ra + b1^2) = sqrt(rb + (b - b1)^2)
			// ra + b1^2 = rb + (b - b1)^2
			// ra + b1^2 = rb + b^2 - 2*b*b1 * b1^2
			// ra = rb + b^2 - 2*b*b1
			// b1 * 2b = rb + b^2 - ra
			assert(b != 0);
			return { (rb + b * b - ra) / (2 * b) };
		}

		// d^2 + 2d * sqrt(ra + b1^2) + ra + b1^2 = rb + (b - b1)^2 // good
		// sqrt(ra + b1^2) = (rb + (b - b1)^2 - d^2 - ra - b1^2) / 2d
		// ra + b1^2 = (rb + (b - b1)^2 - d^2 - ra - b1^2)^2 / (4d^2)
		// 4d^2 * (ra + b1^2) = (rb + (b - b1)^2 - d^2 - ra - b1^2) ^ 2
		// 4d^2 * (ra + b1^2) = (rb + b^2 - 2*b*b1 + b1^2 - d^2 - ra - b1^2) ^ 2
		// 4d^2 * (ra + b1^2) = (rb + b^2 - 2*b*b1 - d^2 - ra) ^ 2
		auto m = rb + b*b - d*d - ra;
		// 4d^2 * (ra + b1^2) = (m - 2*b*b1) ^ 2
		// 4d^2 * (ra + b1^2) = m^2 - 4*m*b*b1 + 4*b^2*b1^2
		// 0 = b1^2 * (4b^2 - 4d^2) + b1 * (-4mb) + (m^2 - 4d^2 * ra)
		auto result = quadraticFormula(4 * b * b - 4 * d * d, -4 * m * b, m * m - 4 * d * d * ra);
		auto endIterator = std::copy_if(result.begin(), result.end(), result.begin(), [&](Kernel::FT b1) {
			return (rb + (b - b1) * (b - b1) - d * d - ra - b1 * b1) / (2 * d) >= 0 &&
				d + sqrt(ra + b1 *b1) >= 0;
		});
		result.erase(endIterator, result.end());
		return result;
	}

	std::vector<Kernel::FT> calculateTiePoints(const CandidateInterval& a, const CandidateInterval& b) {
		assert(a.getHalfedge() == b.getHalfedge());
		auto halfedge = a.getHalfedge();
		auto source = halfedge->opposite()->vertex()->point();
		auto destination = halfedge->vertex()->point();
		auto vector = destination - source;

		auto d1 = a.getDepth();
		auto projectionScalarA = projectionScalar(halfedge, a.getUnfoldedRoot());
		auto closestApproachA = source + projectionScalarA * vector;
		auto r1 = distanceBetweenPoints(a.getUnfoldedRoot(), closestApproachA);
		auto projectionScalarB = projectionScalar(halfedge, b.getUnfoldedRoot());
		auto closestApproachB = source + projectionScalarB * vector;
		auto distance = distanceBetweenPoints(closestApproachA, closestApproachB);
		auto d2 = b.getDepth();
		auto r2 = distanceBetweenPoints(b.getUnfoldedRoot(), closestApproachB);

		auto result = calculateTiePoints(d1, r1, distance, d2, r2);
		// The result is distances, but we need scalar multipliers
		std::transform(result.begin(), result.end(), result.begin(), [&](Kernel::FT tiePointDistance) {
			return projectionScalarA + distance / std::sqrt(vector.squared_length());
		});
		return result;
	}

	void insertInterval(CandidateInterval& interval, std::vector<std::list<CandidateInterval>::iterator>& intervals, const CandidateInterval& predecessor, std::function<std::list<CandidateInterval>::iterator(CandidateInterval)> addCandidateInterval) {
		CandidateInterval::AccessPoint searchFor = { 1 - predecessor.frontierPoint, predecessor.getHalfedge()->opposite() == interval.getHalfedge()->next() };

		auto searchComparison = [](const CandidateInterval::AccessPoint probe, const std::list<CandidateInterval>::iterator& existing) {
			if (existing->accessPoint.initialSide && !probe.initialSide)
				return true;
			if (!existing->accessPoint.initialSide && probe.initialSide)
				return false;
			return existing->accessPoint.location < probe.location;
		};

		auto location = std::upper_bound(intervals.begin(), intervals.end(), searchFor, searchComparison);

		auto findEndOfDominatedIntervals = [&](auto begin, auto end, Polyhedron::Point_3(CandidateInterval::*extentFunction)() const) {
			while (begin != end) {
				if ((*begin)->lowerExtent >= interval.lowerExtent && (*begin)->upperExtent <= interval.upperExtent &&
					distanceBetweenPoints(((**begin).*extentFunction)(), (*begin)->getUnfoldedRoot()) + (*begin)->getDepth() >=
					distanceBetweenPoints(((**begin).*extentFunction)(), interval.getUnfoldedRoot()) + interval.getDepth())
					++begin;
				else
					break;
			}
			return begin;
		};

		auto beginDeleting = findEndOfDominatedIntervals(std::make_reverse_iterator(location), intervals.rend(), &CandidateInterval::getLowerExtent);
		auto endDeleting = findEndOfDominatedIntervals(location, intervals.end(), &CandidateInterval::getUpperExtent);
		// These are pointing to the furthest item which should NOT be deleted.
		// Calling base() on beginDeleting will make it point to the first item which SHOULD be deleted. (Or equal to endDeleting.)

		// "Left and "right" are in reference to a halfedge which points to the right. (--------->)
		boost::optional<Kernel::FT> leftTrimPoint;
		boost::optional<Kernel::FT> rightTrimPoint;

		auto calculateTrimPoints = [](const CandidateInterval& a, const CandidateInterval& b) -> boost::optional<Kernel::FT> {
			auto tiePoints = calculateTiePoints(a, b);
			auto endIterator = std::copy_if(tiePoints.begin(), tiePoints.end(), tiePoints.begin(), [&](Kernel::FT scalar) {
				return scalar >= a.lowerExtent && scalar >= a.upperExtent &&
					scalar >= b.lowerExtent && scalar <= b.upperExtent;
			});
			tiePoints.erase(endIterator, tiePoints.end());
			if (tiePoints.empty())
				return boost::none;
			else {
				assert(tiePoints.size() == 1);
				return tiePoints[0];
			}
		};

		if (beginDeleting != intervals.rend()) {
			if (auto trimPoint = calculateTrimPoints(**beginDeleting, interval))
				leftTrimPoint = trimPoint;
			else
				return;
		}

		if (endDeleting != intervals.end()) {
			if (auto trimPoint = calculateTrimPoints(interval, **endDeleting))
				rightTrimPoint = trimPoint;
			else
				return;
		}

		if (leftTrimPoint && rightTrimPoint && *leftTrimPoint > *rightTrimPoint)
			return;

		// Ready to go! Let's start modifying things.

		// The event queue has iterators into the candidateIntervals list, so we can't simply delete them without updating the event queue.
		std::for_each(beginDeleting.base(), endDeleting, [](auto candidateInterval) {
			candidateInterval->setDeleted();
		});
		bool result = beginDeleting == intervals.rend() || endDeleting == intervals.end();

		auto insertLocation = intervals.erase(beginDeleting.base(), endDeleting);

		auto trimRightSide = [](const CandidateInterval& interval, Kernel::FT location) {
			return CandidateInterval(interval.getHalfedge(), interval.getUnfoldedRoot(), interval.getDepth(), interval.lowerExtent, location);
		};

		auto trimLeftSide = [](const CandidateInterval& interval, Kernel::FT location) {
			return CandidateInterval(interval.getHalfedge(), interval.getUnfoldedRoot(), interval.getDepth(), location, interval.upperExtent);
		};

		auto trimLeftNeighbor = [&](std::vector<std::list<CandidateInterval>::iterator>::iterator location) {
			if (!leftTrimPoint)
				return location;

			auto leftNeighbor = location;
			--leftNeighbor;

			if ((*leftNeighbor)->frontierPoint > *leftTrimPoint) {
				(*leftNeighbor)->setDeleted();
				auto newLeftSide = trimRightSide(**leftNeighbor, *leftTrimPoint);
				auto addedLeftSide = addCandidateInterval(newLeftSide);
				auto insertLocation = intervals.erase(leftNeighbor);
				auto result = intervals.insert(insertLocation, addedLeftSide);
				++result;
				return result;
			}
			else {
				(*leftNeighbor)->upperExtent = *leftTrimPoint;
				return location;
			}
		};

		insertLocation = trimLeftNeighbor(insertLocation);

		auto trimRightNeighbor = [&](std::vector<std::list<CandidateInterval>::iterator>::iterator location) {
			if (!rightTrimPoint)
				return location;

			auto rightNeighbor = location;

			if ((*rightNeighbor)->frontierPoint < *rightTrimPoint) {
				(*rightNeighbor)->setDeleted();
				auto newRightSide = trimLeftSide(**rightNeighbor, *rightTrimPoint);
				auto addedRightSide = addCandidateInterval(newRightSide);
				auto insertLocation = intervals.erase(rightNeighbor);
				auto result = intervals.insert(insertLocation, addedRightSide);
				return result;
			}
			else {
				(*rightNeighbor)->lowerExtent = *rightTrimPoint;
				return location;
			}
		};

		insertLocation = trimRightNeighbor(insertLocation);

		if ((rightTrimPoint && interval.frontierPoint > *rightTrimPoint) ||
			(leftTrimPoint && interval.frontierPoint < *leftTrimPoint)) {
			CandidateInterval newInterval = interval;
			if (rightTrimPoint && leftTrimPoint)
				newInterval = trimLeftSide(trimRightSide(newInterval, *rightTrimPoint), *leftTrimPoint);
			else if (rightTrimPoint)
				newInterval = trimRightSide(newInterval, *rightTrimPoint);
			else if (leftTrimPoint)
				newInterval = trimLeftSide(newInterval, *leftTrimPoint);
			auto addedInterval = addCandidateInterval(newInterval);
			intervals.insert(insertLocation, addedInterval);
		}
		else {
			if (leftTrimPoint)
				interval.lowerExtent = *leftTrimPoint;
			if (rightTrimPoint)
				interval.upperExtent = *rightTrimPoint;
			auto addedInterval = addCandidateInterval(interval);
			intervals.insert(insertLocation, addedInterval);
		}
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
		frontierPoint = projectionScalar(halfedge, unfoldedRoot);

		auto source = halfedge->opposite()->vertex()->point();
		auto destination = halfedge->vertex()->point();
		std::cout << "Candidate Interval on halfedge ";
		printPoint(std::cout, source);
		std::cout << " -> ";
		printPoint(std::cout, destination);
		std::cout << " frontierPoint: " << frontierPoint << std::endl;

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
		if (!initialSideFraction || *initialSideFraction > 1 || *initialSideFraction < 0) {
			auto secondarySideFraction = lineLineIntersection(halfedge->next()->vertex()->point(), halfedge->next()->next()->vertex()->point(), unfoldedRoot, getFrontierPoint());
			if (!initialSideFraction && secondarySideFraction)
				return { std::min(std::max(*secondarySideFraction, Kernel::FT(0)), Kernel::FT(1)), false };
			else if (initialSideFraction && !secondarySideFraction)
				return { std::min(std::max(*initialSideFraction, Kernel::FT(0)), Kernel::FT(1)), true };
			else {
				assert(initialSideFraction && secondarySideFraction);
				if (*secondarySideFraction > 1 || *secondarySideFraction < 0) {
					auto initialSideDistance = std::min(std::abs(*initialSideFraction), std::abs(*initialSideFraction - 1));
					auto secondarySideDistance = std::min(std::abs(*secondarySideFraction), std::abs(*secondarySideFraction - 1));
					if (initialSideDistance < secondarySideDistance)
						return { std::min(std::max(*initialSideFraction, Kernel::FT(0)), Kernel::FT(1)), true };
					else
						return { std::min(std::max(*secondarySideFraction, Kernel::FT(0)), Kernel::FT(1)), false };
				}
				else
					return { *secondarySideFraction, false };
			}
		}
		else
			return { *initialSideFraction, true };
	}

	Kernel::Point_3 CandidateInterval::getFrontierPoint() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		auto s = destination - source;
		return source + frontierPoint * s;
	}

	Kernel::Point_3 CandidateInterval::getLowerExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		auto s = destination - source;
		return source + lowerExtent * s;
	}

	Kernel::Point_3 CandidateInterval::getUpperExtent() const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		auto s = destination - source;
		return source + upperExtent * s;
	}

	// Precondition: Point is part of the candidate interval.
	static Kernel::FT computeLabel(Kernel::Point_3 point, const CandidateInterval& candidateInterval) {
		auto unfoldedRoot = candidateInterval.getUnfoldedRoot();
		auto displacement = point - unfoldedRoot;
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

	boost::optional<Kernel::FT> lineLineIntersection(Polyhedron::Point_3 a1, Polyhedron::Point_3 a2, Polyhedron::Point_3 b1, Polyhedron::Point_3 b2) {
		// Calculates the fraction of the distance between a1 and a2 the intersection occurs.
		auto av = a2 - a1;
		auto bv = b2 - b1;

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

		if (denominatorXY == 0 && denominatorXZ == 0 && denominatorYZ == 0)
			return boost::none;

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
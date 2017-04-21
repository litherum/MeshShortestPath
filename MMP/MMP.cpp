#include "pch.h"
#include "MMP.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <cassert>
#include <algorithm>
#include <list>
#include <cmath>

typedef CGAL::Simple_cartesian<double> Kernel;

class CandidateInterval;


struct AccessPoint {
	Kernel::FT location;
	bool initialSide;
};

template <class Refs>
class MMPHalfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {
public:
	void insertInterval(CandidateInterval& interval, std::function<std::list<CandidateInterval>::iterator(CandidateInterval)> addCandidateInterval) {
		// FIXME: It's not clear to me that we maintain the invariant that the access channel bounds never cross. Maybe this is why the "> 180deg" clause is in the algorithm?
		AccessPoint searchFor = { 0, false };
		insertInterval(interval, searchFor, addCandidateInterval);
	}

	void insertInterval(CandidateInterval& interval, const CandidateInterval& predecessor, std::function<std::list<CandidateInterval>::iterator(CandidateInterval)> addCandidateInterval) {
		std::ostringstream stream;
		auto start = opposite()->vertex()->point();
		auto end = vertex()->point();
		stream << "Inserting into halfedge (" << start.x() << ", " << start.y() << ", " << start.z() << ") -> (" << end.x() << ", " << end.y() << ", " << end.z() << ")" << std::endl;
		stream << "Predecessor info: " << predecessor.frontierPoint << " " << (predecessor.getHalfedge()->opposite() == interval.getHalfedge()->next()) << std::endl;
		for (auto& interval : intervals) {
			stream << "Interval: " << interval->getLowerExtentFraction() << " - " << interval->getUpperExtentFraction() << " Frontier Point: " << interval->frontierPoint << std::endl;
		}
		OutputDebugStringA(stream.str().c_str());

		AccessPoint searchFor = { 1 - predecessor.frontierPoint, predecessor.getHalfedge()->opposite() == interval.getHalfedge()->next() };
		insertInterval(interval, searchFor, addCandidateInterval);
	}

	void insertInterval(CandidateInterval& interval, typename AccessPoint searchFor, std::function<std::list<CandidateInterval>::iterator(CandidateInterval)> addCandidateInterval) {
		auto searchComparison = [](const AccessPoint probe, const std::list<CandidateInterval>::iterator& existing) {
			if (existing->accessPoint.initialSide && !probe.initialSide)
				return true;
			if (!existing->accessPoint.initialSide && probe.initialSide)
				return false;
			return existing->accessPoint.location < probe.location;
		};

		auto location = std::upper_bound(intervals.begin(), intervals.end(), searchFor, searchComparison);

		auto findEndOfDominatedIntervals = [&](auto begin, auto end, Polyhedron::Point(CandidateInterval::*extentFunction)() const) {
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

		std::ostringstream stream;
		stream << (location == intervals.end()) << std::endl;
		OutputDebugStringA(stream.str().c_str());

		auto beforeSide = std::make_reverse_iterator(location);
		auto afterSide = location;

		// Maybe? Perhaps I should be comparing the access points and the predecessor's frontier point.
		if (beforeSide != intervals.rend()) {
			if (std::abs((*beforeSide)->frontierPoint - interval.frontierPoint) < 0.001) {
				OutputDebugStringA("broken");
			}
		}
		if (afterSide != intervals.end()) {
			if (std::abs((*afterSide)->frontierPoint - interval.frontierPoint) < 0.001) {
				OutputDebugStringA("broken");
			}
		}

		auto beginDeleting = findEndOfDominatedIntervals(beforeSide, intervals.rend(), &CandidateInterval::getLowerExtent);
		auto endDeleting = findEndOfDominatedIntervals(afterSide, intervals.end(), &CandidateInterval::getUpperExtent);
		// These are pointing to the furthest item which should NOT be deleted.
		// Calling base() on beginDeleting will make it point to the first item which SHOULD be deleted. (Or equal to endDeleting.)

		// "Left and "right" are in reference to a halfedge which points to the right. (--------->)
		boost::optional<Kernel::FT> leftTrimPoint;
		boost::optional<Kernel::FT> rightTrimPoint;

		auto calculateTrimPoints = [](const CandidateInterval& a, const CandidateInterval& b) -> boost::optional<Kernel::FT> {
			auto tiePoints = calculateTiePoints(a, b);
			auto endIterator = std::copy_if(tiePoints.begin(), tiePoints.end(), tiePoints.begin(), [&](Kernel::FT scalar) {
				return scalar >= a.lowerExtent && scalar <= a.upperExtent &&
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
			return CandidateInterval(interval.getHalfedge(), interval.getUnfoldedRoot(), interval.getDepth(), interval.lowerExtent, location, interval.getIsFromSaddlePoint());
		};

		auto trimLeftSide = [](const CandidateInterval& interval, Kernel::FT location) {
			return CandidateInterval(interval.getHalfedge(), interval.getUnfoldedRoot(), interval.getDepth(), location, interval.upperExtent, interval.getIsFromSaddlePoint());
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

	void insertInitialInterval(std::list<CandidateInterval>::iterator interval) {
		assert(intervals.empty());
		intervals.push_back(interval);
	}

	void iterateIntervals(std::function<void(const CandidateInterval&)> iterator) const {
		for (auto interval : intervals) {
			iterator(*interval);
		}
	}

	uint8_t getIndex() const {
		return index;
	}

	void setIndex(uint8_t index) {
		this->index = index;
	}

private:
	std::vector<std::list<CandidateInterval>::iterator> intervals;
	uint8_t index; // 0 means "pointing from vertex 0 to vertex 1"
};

template <class Refs, class Traits>
class MMPFace : public CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_true, typename Traits::Plane_3> {
public:
	MMP::TriangleIndices::size_type getIndex() const {
		return index;
	}

	void setIndex(MMP::TriangleIndices::size_type index) {
		this->index = index;
	}

private:
	MMP::TriangleIndices::size_type index;
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef MMPHalfedge<Refs> Halfedge;
	};
	template <class Refs, class Traits>
	struct Face_wrapper {
		typedef typename Traits::Plane_3 Plane;
		typedef MMPFace<Refs, Traits> Face;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;

static Kernel::FT distanceBetweenPoints(Polyhedron::Point a, Polyhedron::Point b) {
	return std::sqrt((b - a).squared_length());
}

static Kernel::FT projectionScalar(Polyhedron::Halfedge_handle halfedge, Polyhedron::Point point) {
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
		return {(-b + discriminantSquareRoot) / (2 * a), (-b - discriminantSquareRoot) / (2 * a)};
	}
}

static bool checkLineLineIntersectionResult(Kernel::FT result, Polyhedron::Point p1, Kernel::Vector_3 v1, Polyhedron::Point p2, Kernel::Vector_3 v2) {
	auto resultPoint = p1 + result * v1;
	Kernel::Line_3 l2(p2, v2);
	auto projected = l2.projection(resultPoint);
	auto displacement = projected - resultPoint;
	return displacement.squared_length() <= 0.001;
}

static bool areCoplanar(Polyhedron::Point a, Polyhedron::Point b, Polyhedron::Point c, Polyhedron::Point d) {
	if (distanceBetweenPoints(a, b) < 0.001 || distanceBetweenPoints(a, c) < 0.001 || distanceBetweenPoints(a, d) < 0.001 || distanceBetweenPoints(b, c) < 0.001 || distanceBetweenPoints(b, d) < 0.001 || distanceBetweenPoints(c, d) < 0.001)
		return true;

	Kernel::Line_3 line(a, b);
	auto projected = line.projection(c);
	auto distance = std::sqrt((projected - c).squared_length());
	if (distance < 0.001)
		return true;

	Polyhedron::Plane_3 plane(a, b, c);
	projected = plane.projection(d);
	distance = std::sqrt((d - projected).squared_length());
	return distance < 0.001;
}

static boost::optional<Kernel::FT> lineLineIntersection(Polyhedron::Point a1, Polyhedron::Point a2, Polyhedron::Point b1, Polyhedron::Point b2) {
	assert(areCoplanar(a1, a2, b1, b2));
	assert(areCoplanar(a1, a2, b2, b1));
	assert(areCoplanar(a1, b1, a2, b2));
	assert(areCoplanar(a1, b1, b2, a2));
	assert(areCoplanar(a1, b2, a2, b1));
	assert(areCoplanar(a1, b2, b1, a2));

	assert(areCoplanar(a2, a1, b1, b2));
	assert(areCoplanar(a2, a1, b2, b1));
	assert(areCoplanar(a2, b1, a1, b2));
	assert(areCoplanar(a2, b1, b2, a1));
	assert(areCoplanar(a2, b2, a1, b1));
	assert(areCoplanar(a2, b2, b1, a1));

	assert(areCoplanar(b1, a1, a2, b2));
	assert(areCoplanar(b1, a1, b2, a2));
	assert(areCoplanar(b1, a2, a1, b2));
	assert(areCoplanar(b1, a2, b2, a1));
	assert(areCoplanar(b1, b2, a1, a2));
	assert(areCoplanar(b1, b2, a2, a1));

	assert(areCoplanar(b2, a1, a2, b1));
	assert(areCoplanar(b2, a1, b1, a2));
	assert(areCoplanar(b2, a2, a1, b1));
	assert(areCoplanar(b2, a2, b1, a1));
	assert(areCoplanar(b2, b1, a1, a2));
	assert(areCoplanar(b2, b1, a2, a1));

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

	auto denominatorXYAbs = std::abs(denominatorXY);
	auto denominatorXZAbs = std::abs(denominatorXZ);
	auto denominatorYZAbs = std::abs(denominatorYZ);

	if (denominatorXYAbs < 0.0000001 && denominatorXZAbs < 0.0000001 && denominatorYZAbs < 0.0000001)
		return boost::none;

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

class CandidateInterval {
public:
	CandidateInterval(Polyhedron::Halfedge_handle halfedge, Polyhedron::Point unfoldedRoot, Kernel::FT depth, Kernel::FT lowerExtent, Kernel::FT upperExtent, bool isFromSaddlePoint) : halfedge(halfedge), unfoldedRoot(unfoldedRoot), depth(depth), lowerExtent(lowerExtent), upperExtent(upperExtent), isFromSaddlePoint(isFromSaddlePoint) {
		assert(lowerExtent <= upperExtent);
		assert(lowerExtent >= 0);
		assert(upperExtent <= 1);
		frontierPoint = projectionScalar(halfedge, unfoldedRoot);

		// FIXME: Provide some more resilient way to test for frontier points coincident with vertices
		if (frontierPoint <= lowerExtent) {
			frontierPoint = lowerExtent;
		}
		else if (frontierPoint >= upperExtent) {
			frontierPoint = upperExtent;
		}

		accessPoint = calculateAccessPoint();
	}

	Polyhedron::Point getUnfoldedRoot() const {
		return unfoldedRoot;
	}

	Polyhedron::Halfedge_handle getHalfedge() const {
		return halfedge;
	}

	Kernel::FT getDepth() const {
		return depth;
	}

	Kernel::FT getLowerExtentFraction() const {
		return lowerExtent;
	}

	Kernel::FT getUpperExtentFraction() const {
		return upperExtent;
	}

	Polyhedron::Point getFrontierPoint() const {
		return calculatePoint(frontierPoint);
	}

	Polyhedron::Point getLowerExtent() const {
		return calculatePoint(lowerExtent);
	}

	Polyhedron::Point getUpperExtent() const {
		return calculatePoint(upperExtent);
	}

	enum class FrontierPointLocation {
		SourceVertex,
		DestinationVertex,
		Interior
	};

	FrontierPointLocation getFrontierPointLocation() const {
		// FIXME: Do some sort of rounding.
		if (frontierPoint == 0)
			return FrontierPointLocation::SourceVertex;
		if (frontierPoint == 1)
			return FrontierPointLocation::DestinationVertex;
		return FrontierPointLocation::Interior;
	}

	bool isDeleted() const {
		return deleted;
	}

	void setDeleted() {
		deleted = true;
	}

	bool getIsFromSaddlePoint() const {
		return isFromSaddlePoint;
	}

private:
	template <class Refs>
	friend class MMPHalfedge;

	static Kernel::FT projectionScalar(Polyhedron::Halfedge_handle halfedge, Polyhedron::Point point) {
		auto source = halfedge->opposite()->vertex()->point();
		auto destination = halfedge->vertex()->point();
		auto b = destination - source;
		auto a = point - source;
		return (a * b) / (b * b);
	}

	AccessPoint calculateAccessPoint() const {
		auto initialSideFraction = lineLineIntersection(halfedge->vertex()->point(), halfedge->next()->vertex()->point(), unfoldedRoot, getFrontierPoint());
		if (!initialSideFraction || *initialSideFraction > 1 || *initialSideFraction < 0) {
			auto secondarySideFraction = lineLineIntersection(halfedge->next()->vertex()->point(), halfedge->next()->next()->vertex()->point(), unfoldedRoot, getFrontierPoint());
			if (!initialSideFraction && secondarySideFraction)
				return {std::min(std::max(*secondarySideFraction, Kernel::FT(0)), Kernel::FT(1)), false};
			else if (initialSideFraction && !secondarySideFraction)
				return {std::min(std::max(*initialSideFraction, Kernel::FT(0)), Kernel::FT(1)), true};
			else {
				assert(initialSideFraction && secondarySideFraction);
				if (*secondarySideFraction > 1 || *secondarySideFraction < 0) {
					auto initialSideDistance = std::min(std::abs(*initialSideFraction), std::abs(*initialSideFraction - 1));
					auto secondarySideDistance = std::min(std::abs(*secondarySideFraction), std::abs(*secondarySideFraction - 1));
					if (initialSideDistance < secondarySideDistance)
						return {std::min(std::max(*initialSideFraction, Kernel::FT(0)), Kernel::FT(1)), true};
					else
						return {std::min(std::max(*secondarySideFraction, Kernel::FT(0)), Kernel::FT(1)), false};
				}
				else
					return {*secondarySideFraction, false};
			}
		}
		else
			return {*initialSideFraction, true};
	}

	Polyhedron::Point calculatePoint(Kernel::FT fraction) const {
		auto destination = halfedge->vertex()->point();
		auto source = halfedge->opposite()->vertex()->point();
		auto s = destination - source;
		return source + fraction * s;
	}

	Polyhedron::Point unfoldedRoot;
	Polyhedron::Halfedge_handle halfedge;
	AccessPoint accessPoint;
	Kernel::FT depth;
	Kernel::FT lowerExtent; // 0 <= lowerExtent <= frontierPoint <= upperExtent <= 1
	Kernel::FT upperExtent;
	Kernel::FT frontierPoint;
	bool deleted {false};
	bool isFromSaddlePoint {false};
};

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
		return {(rb + b * b - ra) / (2 * b)};
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

static std::vector<Kernel::FT> calculateTiePoints(const CandidateInterval& a, const CandidateInterval& b) {
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
		return projectionScalarA + tiePointDistance / std::sqrt(vector.squared_length());
	});
	return result;
}

class MMP::Impl {
public:
	// FIXME: This is too restrictive.
	Impl() = delete;
	Impl(const Impl&) = delete;
	Impl(Impl&&) = delete;
	void operator=(const Impl&) = delete;
	void operator=(Impl&&) = delete;

	Impl(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) {
		// FIXME: Make sure that everything is in bounds.
		Generator generator(pointHeap, triangles, startingPointOwningTriangle);
		polyhedron.delegate(generator);
		// FIXME: Make sure we are a manifold.
		startingFacet = generator.getStartingFacet();

		auto startingTriangle = triangles[startingPointOwningTriangle];
		auto p0 = convertPoint(pointHeap[std::get<0>(startingTriangle)]);
		auto p1 = convertPoint(pointHeap[std::get<1>(startingTriangle)]);
		auto p2 = convertPoint(pointHeap[std::get<2>(startingTriangle)]);
		auto v0 = p1 - p0;
		auto v1 = p2 - p0;
		startingPoint = p0 + u * v0 + v * v1;
	}

	void run() {
		auto halfedge = startingFacet->halfedge();
		do {
			CandidateInterval candidateInterval(halfedge, startingPoint, 0, 0, 1, true);
			auto iterator = candidateIntervals.insert(candidateIntervals.end(), candidateInterval);
			halfedge->insertInitialInterval(iterator);
			eventQueue.place(FrontierPointEvent(iterator));
			eventQueue.place(EndPointEvent(halfedge->vertex()->point(), *iterator));
			halfedge = halfedge->next();
		} while (halfedge != startingFacet->halfedge());

		while (!eventQueue.empty()) {
			GenericEvent e = eventQueue.remove();
			boost::apply_visitor(EventVisitor(*this), e);
		}
	}

	std::vector<std::array<std::vector<HalfedgeInterval>, 3>> intervals() const {
		std::ostringstream ss;
		std::vector<std::array<std::vector<HalfedgeInterval>, 3>> result(polyhedron.size_of_facets());
		for (auto facet = polyhedron.facets_begin(); facet != polyhedron.facets_end(); ++facet) {
			ss << "Encountering triangle " << facet->getIndex() << std::endl;
			auto halfedge = facet->halfedge();
			do {
				auto opposite = halfedge->opposite();
				ss << "Halfedge " << static_cast<int>(halfedge->getIndex()) << " opposite triangle: " << opposite->facet()->getIndex() << " opposite halfedge: " << static_cast<int>(opposite->getIndex()) << " Halfedge coordinates: (" << opposite->vertex()->point().x() << ", " << opposite->vertex()->point().y() << ", " << opposite->vertex()->point().z() << ") (" << halfedge->vertex()->point().x() << ", " << halfedge->vertex()->point().y() << ", " << halfedge->vertex()->point().z() << ")" << std::endl;
				std::vector<HalfedgeInterval> oppositeIntervals;
				std::vector<HalfedgeInterval> halfedgeIntervals;
				halfedge->iterateIntervals([&](const CandidateInterval& interval) {
					assert(!interval.isDeleted());
					auto unfoldedRoot = interval.getUnfoldedRoot();
					auto lowerExtent = interval.getLowerExtentFraction();
					auto upperExtent = interval.getUpperExtentFraction();
					auto newUnfoldedRoot = calculateUnfoldedRoot(interval.getUnfoldedRoot(), halfedge->facet()->plane(), opposite->facet()->plane(), Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()));
					oppositeIntervals.push_back({ { newUnfoldedRoot.x(), newUnfoldedRoot.y(), newUnfoldedRoot.z() }, 1 - upperExtent, 1 - lowerExtent, interval.getDepth() });
					if (interval.getIsFromSaddlePoint())
						halfedgeIntervals.push_back({ { unfoldedRoot.x(), unfoldedRoot.y(), unfoldedRoot.z() }, upperExtent, lowerExtent, interval.getDepth() });
				});
				auto& oppositeDestination = result[opposite->facet()->getIndex()][opposite->getIndex()];
				oppositeDestination.insert(oppositeDestination.end(), oppositeIntervals.begin(), oppositeIntervals.end());
				auto& halfedgeDestination = result[halfedge->facet()->getIndex()][halfedge->getIndex()];
				halfedgeDestination.insert(halfedgeDestination.end(), halfedgeIntervals.begin(), halfedgeIntervals.end());
				halfedge = halfedge->next();
			} while (halfedge != facet->halfedge());
		}
		OutputDebugStringA(ss.str().c_str());
		
		return result;
	}

private:
	class Generator : public CGAL::Modifier_base<Polyhedron::HalfedgeDS> {
	public:
		Generator(const MMP::PointHeap& pointHeap, const MMP::TriangleIndices& triangles, MMP::TriangleIndices::size_type startingPointOwningTriangle) : pointHeap(pointHeap), triangles(triangles), startingPointOwningTriangle(startingPointOwningTriangle) {
		}

		Polyhedron::Facet_handle getStartingFacet() {
			return startingFacet;
		}

	private:
		void operator()(Polyhedron::HalfedgeDS& halfedgeDS) {
			CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> builder(halfedgeDS, true);
			builder.begin_surface(pointHeap.size(), triangles.size());
			std::vector<Polyhedron::Vertex_handle> vertices;
			for (MMP::PointHeap::size_type i = 0; i < pointHeap.size(); ++i) {
				const auto& point = pointHeap[i];
				auto vertex = builder.add_vertex(convertPoint(point));
				vertices.push_back(vertex);
			}
			for (MMP::TriangleIndices::size_type i = 0; i < triangles.size(); ++i) {
				const auto& triangle = triangles[i];
				auto facet = builder.begin_facet();
				builder.add_vertex_to_facet(std::get<0>(triangle));
				builder.add_vertex_to_facet(std::get<1>(triangle));
				builder.add_vertex_to_facet(std::get<2>(triangle));
				auto halfedge = builder.end_facet();

				assert(halfedge->vertex() == vertices[std::get<0>(triangle)]);
				assert(halfedge->next()->vertex() == vertices[std::get<1>(triangle)]);
				assert(halfedge->next()->next()->vertex() == vertices[std::get<2>(triangle)]);
				assert(halfedge->next()->next()->next() == halfedge);

				facet->setIndex(i);
				auto p0 = halfedge->vertex()->point();
				auto p1 = halfedge->next()->vertex()->point();
				auto p2 = halfedge->next()->next()->vertex()->point();
				facet->plane() = Polyhedron::Plane_3(p0, p1, p2);
				halfedge->setIndex(2);
				halfedge->next()->setIndex(0);
				halfedge->next()->next()->setIndex(1);
				if (i == startingPointOwningTriangle) {
					startingFacet = facet;
				}
			}
			builder.end_surface();
		}

		const MMP::PointHeap& pointHeap;
		const MMP::TriangleIndices& triangles;
		MMP::TriangleIndices::size_type startingPointOwningTriangle;
		Polyhedron::Facet_handle startingFacet;
	};

	class FrontierPointEvent {
	public:
		FrontierPointEvent(std::list<CandidateInterval>::iterator candidateInterval) : point(candidateInterval->getFrontierPoint()), candidateInterval(candidateInterval) {
		}

		std::list<CandidateInterval>::iterator getCandidateInterval() const {
			return candidateInterval;
		}

		Kernel::FT FrontierPointEvent::getLabel() const {
			return computeLabel(point, *candidateInterval);
		}

	private:
		Polyhedron::Point point;
		std::list<CandidateInterval>::iterator candidateInterval;
	};

	class EndPointEvent {
	public:
		EndPointEvent(Polyhedron::Point point, const CandidateInterval& candidateInterval) : point(point), label(computeLabel(point, candidateInterval)) {
		}

		Kernel::FT getLabel() const {
			return label;
		}

	private:
		Polyhedron::Point point;
		Kernel::FT label;
	};

	typedef boost::variant<FrontierPointEvent, EndPointEvent> GenericEvent;

	class EventQueue {
	public:
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

		bool empty() const {
			return heap.empty();
		}

	private:
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

		std::vector<GenericEvent> heap;
	};

	class EventVisitor {
	public:
		EventVisitor(Impl& impl) : impl(impl) {
		}

		void operator()(const FrontierPointEvent& event) const {
			if (event.getCandidateInterval()->isDeleted()) {
				impl.candidateIntervals.erase(event.getCandidateInterval());
				return;
			}

			std::ostringstream stream;
			auto source = event.getCandidateInterval()->getHalfedge()->opposite()->vertex()->point();
			auto dest = event.getCandidateInterval()->getHalfedge()->vertex()->point();
			auto l = event.getCandidateInterval()->getLowerExtentFraction();
			auto r = event.getCandidateInterval()->getUpperExtentFraction();
			stream << "Encountering candidate interval (" << source.x() << ", " << source.y() << ", " << source.z() << ")";
			stream << " -> (" << dest.x() << ", " << dest.y() << ", " << dest.z() << ") " << l << " " << r << std::endl;
			OutputDebugStringA(stream.str().c_str());

			// FIXME: Possibly label the event
			impl.propagate(*event.getCandidateInterval());
		}

		void operator()(const EndPointEvent& event) const {
			// FIXME: Possibly label the event
		}

	private:
		Impl& impl;
	};

	static Polyhedron::Point convertPoint(const MMP::PointHeap::value_type& point) {
		return Polyhedron::Point(std::get<0>(point), std::get<1>(point), std::get<2>(point));
	};

	static Kernel::FT computeLabel(Polyhedron::Point point, const CandidateInterval& candidateInterval) {
		auto unfoldedRoot = candidateInterval.getUnfoldedRoot();
		auto displacement = point - unfoldedRoot;
		return std::sqrt(displacement.squared_length()) + candidateInterval.getDepth();
	}

	void propagate(const CandidateInterval& interval) {
		auto insertCandidateInterval = [&](CandidateInterval candidateInterval) {
			auto result = candidateIntervals.insert(candidateIntervals.end(), candidateInterval);
			eventQueue.place(FrontierPointEvent(result));
			return result;
		};
		auto insertSaddlePointIntervals = [&](Polyhedron::Vertex_handle vertex) {
			auto depth = interval.getDepth() + std::sqrt((interval.getUnfoldedRoot() - vertex->point()).squared_length());
			auto circulator = vertex->vertex_begin();
			do {
				assert(circulator->vertex() == vertex);
				Polyhedron::Halfedge_handle halfedge = circulator->next()->next();
				assert(halfedge->vertex() != vertex && halfedge->opposite()->vertex() != vertex);
				CandidateInterval candidateInterval(halfedge, vertex->point(), depth, 0, 1, true);
				halfedge->insertInterval(candidateInterval, insertCandidateInterval);
				circulator++;
			} while (circulator != vertex->vertex_begin());
		};

		auto projected = project(interval);

		for (auto& candidateInterval : projected) {
			if (candidateInterval) {
				candidateInterval->getHalfedge()->insertInterval(*candidateInterval, interval, insertCandidateInterval);
			}
		}

		switch (interval.getFrontierPointLocation()) {
		case CandidateInterval::FrontierPointLocation::SourceVertex: {
			auto& vertex = interval.getHalfedge()->opposite()->vertex();
			insertSaddlePointIntervals(vertex);
			break;
		}
		case CandidateInterval::FrontierPointLocation::DestinationVertex: {
			auto& vertex = interval.getHalfedge()->vertex();
			insertSaddlePointIntervals(vertex);
			break;
		}
		case CandidateInterval::FrontierPointLocation::Interior:
			break;
		}
	}

	static bool isCoplanar(Polyhedron::Plane_3 plane, Polyhedron::Point point) {
		auto projected = plane.projection(point);
		auto distance = std::sqrt((point - projected).squared_length());
		return distance < 0.001;
	}

	Polyhedron::Point calculateUnfoldedRoot(Polyhedron::Point oldUnfoldedRoot, Polyhedron::Plane_3 originalPlane, Polyhedron::Plane_3 newPlane, Kernel::Line_3 intersection) const {
		assert(isCoplanar(originalPlane, oldUnfoldedRoot));
		assert(isCoplanar(originalPlane, intersection.point()));
		assert(isCoplanar(newPlane, intersection.point()));

		auto originalNormal = originalPlane.orthogonal_vector();
		originalNormal = originalNormal / std::sqrt(originalNormal.squared_length());

		auto newNormal = newPlane.orthogonal_vector();
		newNormal = newNormal / std::sqrt(newNormal.squared_length());

		auto crossProduct = CGAL::cross_product(originalNormal, newNormal);
		auto dotProduct = originalNormal * newNormal;
		auto crossProductMagnitude = std::sqrt(crossProduct.squared_length());
		if (std::abs(crossProductMagnitude) < 0.001) { // Coplanar
			if (dotProduct > 0) {
				assert(isCoplanar(newPlane, oldUnfoldedRoot));
				return oldUnfoldedRoot;
			}
			else {
				auto projection = intersection.projection(oldUnfoldedRoot);
				auto delta = oldUnfoldedRoot - projection;
				auto result = projection - delta;
				assert(isCoplanar(newPlane, result));
				return result;
			}
		}
		else {
			auto axis = crossProduct / crossProductMagnitude;
			//auto angle = std::acos(dotProduct);

			auto projection = intersection.projection(oldUnfoldedRoot);
			auto translatedUnfoldedRoot = oldUnfoldedRoot - projection;

			Polyhedron::Plane_3 oldOriginPlane(Polyhedron::Point_3(0, 0, 0), originalNormal);
			assert(isCoplanar(oldOriginPlane, Polyhedron::Point_3(0, 0, 0) + translatedUnfoldedRoot));
			// Rodrigues' rotation formula
			auto sinTheta = crossProductMagnitude; // We pretend the angle is 0 <= theta < pi, and use the orientation of the cross product axis to handle the case otherwise
			auto cosTheta = dotProduct;
			auto rotated = translatedUnfoldedRoot * cosTheta + CGAL::cross_product(axis, translatedUnfoldedRoot) * sinTheta + axis * (axis * translatedUnfoldedRoot) * (1 - cosTheta);
			Polyhedron::Plane_3 newOriginPlane(Polyhedron::Point_3(0, 0, 0), newNormal);
			assert(isCoplanar(newOriginPlane, Polyhedron::Point_3(0, 0, 0) + rotated));
			auto rotatedTranslatedBack = projection - Polyhedron::Point_3(0, 0, 0) + rotated;
			auto result = Polyhedron::Point_3(0, 0, 0) + rotatedTranslatedBack;
			assert(isCoplanar(newPlane, result));
			return result;
		}
	}

	static boost::optional<Kernel::FT> lineLineIntersectionWithDirection(Polyhedron::Point a1, Polyhedron::Point a2, Polyhedron::Point b1, Polyhedron::Point b2) {
		auto intersection = lineLineIntersection(a1, a2, b1, b2);
		if (!intersection)
			return boost::none;
		Kernel::FT t = *intersection;

		// Calculate s to see if it is negative.
		// a + t*b = c + s*d
		// s = (a + t*b - c) / d

		auto calculateS = [&](Kernel::FT p1, Kernel::FT v1, Kernel::FT p2, Kernel::FT v2) {
			// a, b, c, d
			return (p1 + t * v1 - p2) / v2;
		};

		auto av = a2 - a1;
		auto bv = b2 - b1;
		auto bvxAbs = std::abs(bv.x());
		auto bvyAbs = std::abs(bv.y());
		auto bvzAbs = std::abs(bv.z());

		Kernel::FT s;

		if (bvxAbs >= bvyAbs && bvxAbs >= bvzAbs)
			s = calculateS(a1.x(), av.x(), b1.x(), bv.x());
		else if (bvyAbs >= bvxAbs && bvyAbs >= bvzAbs)
			s = calculateS(a1.y(), av.y(), b1.y(), bv.y());
		else
			s = calculateS(a1.z(), av.z(), b1.z(), bv.z());

		if (s < 0) {
			if (av * bv > 0)
				return std::numeric_limits<Kernel::FT>::infinity();
			else
				return -std::numeric_limits<Kernel::FT>::infinity();
		}
		else
			return t;
	}

	std::array<boost::optional<CandidateInterval>, 2> project(const CandidateInterval& interval) {
		auto halfedge = interval.getHalfedge();
		auto opposite = halfedge->opposite();
		auto next = opposite->next();
		auto nextnext = next->next();
		assert(nextnext->next() == opposite);

		auto newUnfoldedRoot = calculateUnfoldedRoot(interval.getUnfoldedRoot(), halfedge->facet()->plane(), opposite->facet()->plane(), Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()));

		auto project = [&](Polyhedron::Halfedge_handle halfedge) -> boost::optional<CandidateInterval> {
			auto source = halfedge->opposite()->vertex()->point();
			auto destination = halfedge->vertex()->point();
			auto lowerScalar = lineLineIntersectionWithDirection(source, destination, newUnfoldedRoot, interval.getLowerExtent());
			auto upperScalar = lineLineIntersectionWithDirection(source, destination, newUnfoldedRoot, interval.getUpperExtent());
			if (!lowerScalar) {
				assert(upperScalar);
				if ((destination - source) * (interval.getLowerExtent() - newUnfoldedRoot) > 0)
					lowerScalar = std::numeric_limits<Kernel::FT>::infinity();
				else
					lowerScalar = -std::numeric_limits<Kernel::FT>::infinity();
			}
			if (!upperScalar) {
				assert(lowerScalar);
				if ((destination - source) * (interval.getUpperExtent() - newUnfoldedRoot) > 0)
					upperScalar = std::numeric_limits<Kernel::FT>::infinity();
				else
					upperScalar = -std::numeric_limits<Kernel::FT>::infinity();
			}
			*lowerScalar = std::min(std::max(*lowerScalar, Kernel::FT(0)), Kernel::FT(1));
			*upperScalar = std::min(std::max(*upperScalar, Kernel::FT(0)), Kernel::FT(1));
			if (upperScalar > lowerScalar)
				return CandidateInterval(halfedge, newUnfoldedRoot, interval.getDepth(), *lowerScalar, *upperScalar, false);
			else
				return boost::none;
		};
		return {project(next), project(nextnext)};
	}

	Polyhedron polyhedron;
	std::list<CandidateInterval> candidateIntervals;
	EventQueue eventQueue;
	Polyhedron::Facet_handle startingFacet;
	Polyhedron::Point startingPoint;
};

MMP::MMP(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) : impl(std::make_unique<Impl>(pointHeap, triangles, startingPointOwningTriangle, u, v)) {
}

MMP::~MMP() {
}

void MMP::run() {
	impl->run();
}

auto MMP::intervals() const -> std::vector<std::array<std::vector<HalfedgeInterval>, 3>> {
	return impl->intervals();
}

// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <list>
#include <algorithm>
#include <vector>
#include <cassert>
#include <memory>
#include <array>

#define _USE_MATH_DEFINES 
#include <cmath>

#include "Types.h"
#include "GenerateGeometry.h"

#include <boost/optional.hpp>

namespace MeshShortestPath {

	static void printPoint(std::ostream& stream, Polyhedron::Point point) {
		stream << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
	}

	template <typename T>
	static void iterateHalfedges(Polyhedron::Facet_handle facet, T callback) {
		auto halfedge = facet->halfedge();
		auto halfedge2 = halfedge;
		do {
			halfedge2 = halfedge2->next();
			callback(halfedge2);
		} while (halfedge2 != halfedge);
	}

	static Polyhedron::Point middlePoint(const Polyhedron::Facet_handle facet) {
		std::vector<Polyhedron::Point> points;
		iterateHalfedges(facet, [&](Polyhedron::Halfedge_handle halfedge) {
			points.emplace_back(halfedge->vertex()->point());
		});
		Polyhedron::Point result(0, 0, 0);
		for (auto point : points) {
			result = Polyhedron::Point(result.x() + point.x(), result.y() + point.y(), result.z() + point.z());
		}
		return Polyhedron::Point(result.x() / points.size(), result.y() / points.size(), result.z() / points.size());
	}

	class MMP {
	public:
		// FIXME: The input can either be a point on the interior of a triangle, a point on the interior of an edge, or a vertex.
		MMP(Polyhedron& polyhedron, Polyhedron::Facet_handle initialFacet, Polyhedron::Point pointOnInitialFacet) : polyhedron(polyhedron) {
			std::vector<Polyhedron::Point> initialFacetPoints;
			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				initialFacetPoints.push_back(halfedge->vertex()->point());
			});
			std::cout << "Starting with point ";
			printPoint(std::cout, pointOnInitialFacet);
			std::cout << " on facet ";
			for (auto& point : initialFacetPoints) {
				if (&point != &initialFacetPoints.front())
					std::cout << ", ";
				printPoint(std::cout, point);
			}
			std::cout << std::endl;

			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				CandidateInterval interval(halfedge, pointOnInitialFacet, 0, 0, 1);
				candidateIntervals.emplace_back(std::move(interval));
				std::list<CandidateInterval>::iterator iterator = candidateIntervals.end();
				--iterator;
				assert(&*iterator == &candidateIntervals.back());
				bool firstOrLastOnHalfedge = halfedge->insertInterval(iterator);
				iterator->setFrontierPointIsAtVertex(iterator->getFrontierPointIsAtExtent() && firstOrLastOnHalfedge);
				eventQueue.place(FrontierPointEvent(iterator->getFrontierPoint(), iterator));
				eventQueue.place(EndPointEvent(halfedge->vertex()->point(), *iterator));
			});
		}

		void populate() {
			while (!eventQueue.empty()) {
				GenericEvent e = eventQueue.remove();
				boost::apply_visitor(EventVisitor(*this), e);
			}
		}

		~MMP() {
			// FIXME: Find some way to export the data
			for (auto i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end(); ++i) {
				i->clear();
			}
		}

	private:
		class EventVisitor {
		public:
			EventVisitor(MMP& mmp) : mmp(mmp) {
			}

			void operator()(const FrontierPointEvent& event) const {
				std::cout << "Encountering event with label " << event.getLabel() << std::endl;
				// FIXME: Possibly label the event
				mmp.propagate(*event.getCandidateInterval());
			}

			void operator()(const EndPointEvent& event) const {
				std::cout << "Encountering event with label " << event.getLabel() << std::endl;
				// FIXME: Possibly label the event
			}

		private:
			MMP& mmp;
		};

		void propagate(const CandidateInterval& interval) {
			if (interval.getFrontierPointIsAtVertex()) {
				// FIXME: Find the vertex, then find opposite edges which are greater than 2*pi, then insertInteval()
			}
			else {
				auto projected = project(interval);

				auto insert = [&](const CandidateInterval& candidateInterval) {
					// FIXME: do insertInterval()
				};

				for (auto& candidateInterval : projected) {
					if (candidateInterval)
						insert(*candidateInterval);
				}
			}
		}

		Polyhedron::Point_3 calculateUnfoldedRoot(Polyhedron::Point_3 oldUnfoldedRoot, Polyhedron::Plane_3 originalPlane, Polyhedron::Plane_3 newPlane, Kernel::Line_3 intersection) {
			auto originalNormal = originalPlane.orthogonal_vector();
			originalNormal = originalNormal / std::sqrt(originalNormal.squared_length());

			auto newNormal = newPlane.orthogonal_vector();
			newNormal = newNormal / std::sqrt(newNormal.squared_length());

			Polyhedron::Point_3 newUnfoldedRoot;
			auto crossProduct = CGAL::cross_product(originalNormal, newNormal);
			auto dotProduct = originalNormal * newNormal;
			if (std::abs(crossProduct.squared_length()) < 0.001) { // Coplanar
				if (dotProduct > 0)
					return oldUnfoldedRoot;
				else {
					auto projection = intersection.projection(oldUnfoldedRoot);
					auto delta = oldUnfoldedRoot - projection;
					return projection - delta;
				}
			}
			else {
				auto magnitude = std::sqrt(crossProduct.squared_length());
				auto axis = crossProduct / magnitude;
				auto angle = std::asin(magnitude);
				if (dotProduct < 0)
					angle = M_PI - angle;

				auto projection = intersection.projection(oldUnfoldedRoot);
				auto translatedUnfoldedRoot = oldUnfoldedRoot - projection;

				// Rodrigues' rotation formula
				auto cosTheta = std::cos(angle);
				auto rotated = translatedUnfoldedRoot * cosTheta + CGAL::cross_product(axis, translatedUnfoldedRoot) + axis * (axis * translatedUnfoldedRoot) * (1 - cosTheta);
				auto rotatedTranslatedBack = Kernel::Vector_3(projection.x(), projection.y(), projection.z()) + rotated;
				return Polyhedron::Point_3(rotatedTranslatedBack.x(), rotatedTranslatedBack.y(), rotatedTranslatedBack.z());
			}
		}

		bool checkLineLineIntersectionResult(Kernel::FT result, Polyhedron::Point_3 p1, Kernel::Vector_3 v1, Polyhedron::Point_3 p2, Kernel::Vector_3 v2) {
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

			Kernel::FT result;

			if (denominatorXYAbs >= denominatorXZAbs && denominatorXYAbs >= denominatorYZAbs) {
				// Use XY
				auto numerator = calculateNumerator(a1.x(), a1.y(), b1.x(), b1.y(), bv.x(), bv.y());
				result = numerator / denominatorXY;
			}
			else if (denominatorXZAbs >= denominatorXYAbs && denominatorXZAbs >= denominatorYZAbs) {
				// Use XZ
				auto numerator = calculateNumerator(a1.x(), a1.z(), b1.x(), b1.z(), bv.x(), bv.z());
				result = numerator / denominatorXZ;
			}
			else {
				// Use YZ
				auto numerator = calculateNumerator(a1.y(), a1.z(), b1.y(), b1.z(), bv.y(), bv.z());
				result = numerator / denominatorYZ;
			}
			assert(checkLineLineIntersectionResult(result, a1, av, b1, bv));
			return result;
		}

		std::array<boost::optional<CandidateInterval>, 2> project(const CandidateInterval& interval) {
			auto halfedge = interval.getHalfedge();
			auto opposite = halfedge->opposite();
			auto next = opposite->next();
			auto nextnext = next->next();
			assert(nextnext->next() == opposite);

			auto newUnfoldedRoot = calculateUnfoldedRoot(interval.getUnfoldedRoot(), halfedge->facet()->plane(), opposite->facet()->plane(), Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()));

			auto horizon1 = Kernel::Line_3(newUnfoldedRoot, interval.getLowerExtent());
			auto horizon2 = Kernel::Line_3(newUnfoldedRoot, interval.getUpperExtent());

			auto project = [&](Polyhedron::Halfedge_handle halfedge) -> boost::optional<CandidateInterval> {
				auto line = Kernel::Line_3(halfedge->vertex()->point(), halfedge->opposite()->vertex()->point());
				auto lowerScalar = lineLineIntersection(halfedge->opposite()->vertex()->point(), halfedge->vertex()->point(), newUnfoldedRoot, interval.getLowerExtent());
				auto upperScalar = lineLineIntersection(halfedge->opposite()->vertex()->point(), halfedge->vertex()->point(), newUnfoldedRoot, interval.getUpperExtent());
				assert(upperScalar >= lowerScalar);
				lowerScalar = std::min(std::max(lowerScalar, Kernel::FT(0)), Kernel::FT(1));
				upperScalar = std::min(std::max(upperScalar, Kernel::FT(0)), Kernel::FT(1));
				if (upperScalar > lowerScalar)
					return CandidateInterval(halfedge, newUnfoldedRoot, interval.getDepth(), lowerScalar, upperScalar);
				else
					return boost::none;
			};
			return { project(next), project(nextnext) };
		}

		Polyhedron& polyhedron;
		std::list<CandidateInterval> candidateIntervals;
		EventQueue eventQueue;
	};

}

int main()
{
	MeshShortestPath::Polyhedron polyhedron = MeshShortestPath::generateCube();
	auto facet = polyhedron.facets_begin();
	MeshShortestPath::MMP mmp(polyhedron, facet, middlePoint(facet));
	mmp.populate();
    return 0;
}


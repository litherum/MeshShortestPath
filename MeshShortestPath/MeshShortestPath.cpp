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
				halfedge->insertInitialInterval(iterator);
				iterator->setFrontierPointIsAtVertex(iterator->getFrontierPointIsAtExtent());
				eventQueue.place(FrontierPointEvent(iterator));
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
				if (event.getCandidateInterval()->isDeleted())
					return;

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

				for (auto& candidateInterval : projected) {
					if (candidateInterval) {
						candidateIntervals.push_back(*candidateInterval);
						auto iterator = candidateIntervals.end();
						--iterator;
						auto insertIntervalResult = candidateInterval->getHalfedge()->insertInterval(iterator, interval);
						if (insertIntervalResult) {
							eventQueue.place(FrontierPointEvent(iterator)); // FIXME: Maybe add more things to the event queue
							// FIXME: Check if the frontier point is a vertex
							for (auto item : insertIntervalResult->toRemove)
								item->setDeleted(); // The event queue has iterators into the candidateIntervals list, so we can't simply delete them without updating the event queue.
						}
						else
							candidateIntervals.erase(iterator);
					}
				}
			}
		}

		Polyhedron::Point_3 calculateUnfoldedRoot(Polyhedron::Point_3 oldUnfoldedRoot, Polyhedron::Plane_3 originalPlane, Polyhedron::Plane_3 newPlane, Kernel::Line_3 intersection) {
			std::cout << "Calculating unfolded root between two planes: " << std::endl;
			std::cout << originalPlane << std::endl;
			std::cout << newPlane << std::endl;
			std::cout << intersection << std::endl;
			std::cout << oldUnfoldedRoot << std::endl;
			std::cout << "End of dump" << std::endl;


			auto originalNormal = originalPlane.orthogonal_vector();
			originalNormal = originalNormal / std::sqrt(originalNormal.squared_length());

			auto newNormal = newPlane.orthogonal_vector();
			newNormal = newNormal / std::sqrt(newNormal.squared_length());

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

		Kernel::FT lineLineIntersectionWithDirection(Polyhedron::Point_3 a1, Polyhedron::Point_3 a2, Polyhedron::Point_3 b1, Polyhedron::Point_3 b2) {
			Kernel::FT t = lineLineIntersection(a1, a2, b1, b2);

			// Calculate s to see if it is negative.
			// a + t*b = c + s*d
			// s = (a + t*b - c) / d

			auto calculateS = [&](Kernel::FT p1, Kernel::FT v1, Kernel::FT p2, Kernel::FT v2) {
				// a, b, c, d
				return (p1 + t * v1 - p2) / v2;
			};

			Kernel::Vector_3 av(a2.x() - a1.x(), a2.y() - a1.y(), a2.z() - a1.z());
			Kernel::Vector_3 bv(b2.x() - b1.x(), b2.y() - b1.y(), b2.z() - b1.z());
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

			std::cout << "Projecting halfedge " << opposite->vertex()->point() << " " << halfedge->vertex()->point() << std::endl;
			std::cout << "Onto " << next->vertex()->point() << std::endl;

			auto newUnfoldedRoot = calculateUnfoldedRoot(interval.getUnfoldedRoot(), halfedge->facet()->plane(), opposite->facet()->plane(), Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()));

			auto project = [&](Polyhedron::Halfedge_handle halfedge) -> boost::optional<CandidateInterval> {
				auto lowerScalar = lineLineIntersectionWithDirection(halfedge->opposite()->vertex()->point(), halfedge->vertex()->point(), newUnfoldedRoot, interval.getLowerExtent());
				auto upperScalar = lineLineIntersectionWithDirection(halfedge->opposite()->vertex()->point(), halfedge->vertex()->point(), newUnfoldedRoot, interval.getUpperExtent());
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


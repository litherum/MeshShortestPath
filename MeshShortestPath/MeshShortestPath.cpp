// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <list>
#include <algorithm>
#include <vector>
#include <cassert>
#include <memory>
#include <array>

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

				if (auto& candidateInterval = std::get<0>(projected))
					insert(*candidateInterval);
				if (auto& candidateInterval = std::get<1>(projected))
					insert(*candidateInterval);
			}
		}

		std::array<boost::optional<CandidateInterval>, 2> project(const CandidateInterval& interval) {
			auto halfedge = interval.getHalfedge();
			auto opposite = halfedge->opposite();
			auto next = opposite->next();
			auto nextnext = next->next();
			assert(nextnext->next() == opposite);

			printPoint(std::cout, opposite->vertex()->point());
			printPoint(std::cout, next->vertex()->point());
			printPoint(std::cout, nextnext->vertex()->point());
			std::cout << std::endl;

			auto originalPlane = halfedge->facet()->plane();
			auto originalNormal = originalPlane.orthogonal_vector();
			originalNormal = originalNormal / std::sqrt(originalNormal.squared_length());

			auto newPlane = opposite->facet()->plane();
			auto newNormal = newPlane.orthogonal_vector();
			newNormal = newNormal / std::sqrt(newNormal.squared_length());

			Polyhedron::Point_3 newUnfoldedRoot;
			auto crossProduct = CGAL::cross_product(originalNormal, newNormal);
			if (crossProduct.squared_length() < 0.001) { // Coplanar
				if (originalNormal * newNormal > 0)
					newUnfoldedRoot = interval.getUnfoldedRoot();
				else {
					auto unfoldedRoot = interval.getUnfoldedRoot();
					auto projection = Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()).projection(unfoldedRoot);
					auto delta = unfoldedRoot - projection;
					newUnfoldedRoot = projection - delta;
				}
			}
			else {
				auto magnitude = std::sqrt(crossProduct.squared_length());
				auto axis = crossProduct / magnitude;
				auto angle = std::asin(magnitude);

				auto unfoldedRoot = interval.getUnfoldedRoot();
				auto projection = Kernel::Line_3(halfedge->vertex()->point(), opposite->vertex()->point()).projection(unfoldedRoot);
				auto translatedUnfoldedRoot = unfoldedRoot - projection;

				// Rodrigues' rotation formula
				auto cosTheta = std::cos(angle);
				auto rotated = translatedUnfoldedRoot * cosTheta + CGAL::cross_product(axis, translatedUnfoldedRoot) + axis * (axis * translatedUnfoldedRoot) * (1 - cosTheta);
				auto rotatedTranslatedBack = Kernel::Vector_3(projection.x(), projection.y(), projection.z()) + rotated;
				newUnfoldedRoot = Polyhedron::Point_3(rotatedTranslatedBack.x(), rotatedTranslatedBack.y(), rotatedTranslatedBack.z());
			}

			auto project1 = [&](Polyhedron::Halfedge_handle halfedge) -> boost::optional<CandidateInterval> {
				return CandidateInterval(halfedge, newUnfoldedRoot, interval.getDepth(), 0, 1); // FIXME: Implement the extents
			};
			return { project1(next), project1(nextnext) };
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


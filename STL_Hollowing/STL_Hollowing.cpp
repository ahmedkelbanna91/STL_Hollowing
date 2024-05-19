#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

#include <CGAL/Alpha_wrap_3.h>
#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <CGAL/IO/STL.h>
#include <vector>
#include <cmath>
#include <numbers>
#include <fstream>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

namespace PMP = CGAL::Polygon_mesh_processing;

Mesh shrink_mesh(Mesh& mesh, double alpha, double offset) {

    CGAL::Polygon_mesh_processing::triangulate_faces(mesh);
    Mesh shrunk_mesh;
    CGAL::alpha_wrap_3(mesh, alpha, offset, shrunk_mesh);

    return shrunk_mesh;
}

Point compute_centroid(const Mesh& mesh) {
    std::vector<Point> vertices;
    for (auto v : mesh.vertices()) {
        vertices.push_back(mesh.point(v));
    }
    return CGAL::centroid(vertices.begin(), vertices.end());
}

void translate_mesh(Mesh& mesh, const Vector& translation_vector) {
    std::cout << "      Applying translation: " << translation_vector << std::endl;
    for (auto v : mesh.vertices()) {
        mesh.point(v) = mesh.point(v) + translation_vector;
    }
}

bool read_STL(const std::string& filename, Mesh& mesh) {
    std::ifstream input(filename, std::ios::binary);
    if (!input) {
        std::cerr << "Error: Cannot open the STL file " << filename << std::endl;
        return false;
    }
    if (!CGAL::IO::read_STL(input, mesh)) {
        std::cerr << "Error: Cannot read the STL file " << filename << std::endl;
        return false;
    }
    return true;
}

bool write_STL(const std::string& filename, const Mesh& mesh) {
    std::ofstream output(filename, std::ios::binary);
    if (!output) {
        std::cerr << "Error: Cannot open the STL file to write " << filename << std::endl;
        return false;
    }

    if (!CGAL::is_valid_polygon_mesh(mesh)) {
        std::cerr << "Error: Mesh is not valid." << std::endl;
    }
    output.precision(12);
    CGAL::IO::set_binary_mode(output);
    if (CGAL::IO::write_STL(output, mesh)) {
        output.flush();  // Flush the stream to ensure all data is written
        output.close();  // Close the file
        return true;
    }
    else {
        std::cerr << "Error: Cannot write the STL file: " << filename << std::endl;
        output.close();  // Ensure file is closed even on failure
        return false;
    }
    return true;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " input.stl output.stl" << std::endl;
        return 1;
    }

    Mesh mesh, hexagon, offset_mesh, hollowed_mesh;
    
    if (!read_STL(argv[1], mesh)) {
        return EXIT_FAILURE;
    }

    // Orienting poly3 and computing its centroid
    CGAL::Polygon_mesh_processing::orient(mesh);
    Point centroid = compute_centroid(mesh);
    Kernel::Vector_3 translation(-centroid.x(), -centroid.y(), 0); // Adjust Z as well if needed
    translate_mesh(mesh, translation);

    double alpha = std::stod(argv[3]);  // Control the level of detail (1)
    double offset = std::stod(argv[4]);  // Negative for shrinking (1)
    offset_mesh = shrink_mesh(mesh, alpha, offset);

    //// Assuming you have Boolean operations set up (e.g., via CGAL's Nef_polyhedron or Corefinement)
    //if (!PMP::corefine_and_compute_difference(mesh, offset_mesh, hollowed_mesh)) {
    //    std::cerr << "Failed to compute the hollowed mesh." << std::endl;
    //    return EXIT_FAILURE;
    //}

    //// Optionally repair the mesh
    //PMP::remove_isolated_vertices(offset_mesh);
    if (!write_STL(argv[2], offset_mesh)) {
        return EXIT_FAILURE;
    }

    std::cout << "Hollowed mesh has been saved successfully." << std::endl;
    return EXIT_SUCCESS;
}
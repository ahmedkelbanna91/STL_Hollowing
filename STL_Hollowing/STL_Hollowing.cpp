//#define CGAL_TRACE_STREAM std::cerr
#define CGAL_DEBUG std::cerr
//#define CGAL_PMP_COMPUTE_NORMAL_DEBUG std::cerr

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Alpha_wrap_3.h>

#include <CGAL/Surface_mesh.h>
#include <vector>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Plane_3 Plane;

namespace PMP = CGAL::Polygon_mesh_processing;

bool slice_mesh(Mesh& mesh, double z_coord) {
    std::cout << "      Slicing Mesh." << std::endl;
    Plane plane(0, 0, -1, z_coord);
    PMP::clip(mesh, plane);
    PMP::stitch_borders(mesh);
    
    return !mesh.is_empty();
}

void apply_inward_offset(Mesh& mesh, double offset_distance) {
    auto normals = mesh.add_property_map<Mesh::Vertex_index, Vector>("v:normals", CGAL::NULL_VECTOR).first;
    PMP::compute_vertex_normals(mesh, normals);
    for (auto v : mesh.vertices()) {
        const Point& p = mesh.point(v);
        const Vector& n = normals[v];
        mesh.point(v) = p - n * offset_distance;  // Move vertex inward by offset_distance
    }
}

void shrink_mesh(Mesh& mesh, double alpha, double offset, Mesh& shrunk_mesh) {
    std::cout << "      Applying Uniform Scaling." << std::endl;
    CGAL::Polygon_mesh_processing::triangulate_faces(mesh);
    CGAL::alpha_wrap_3(mesh, alpha, offset, shrunk_mesh);
}

void compute_centroid(Mesh& mesh, Point& centroid) {
    std::cout << "      Computing Centroid." << std::endl;
    std::vector<Point> vertices;
    for (auto v : mesh.vertices()) {
        vertices.push_back(mesh.point(v));
    }
    centroid = CGAL::centroid(vertices.begin(), vertices.end());
}

void translate_mesh(Mesh& mesh, const Vector& translation_vector) {
    std::cout << "      Applying translation: " << translation_vector << std::endl;
    for (auto v : mesh.vertices()) {
        mesh.point(v) = mesh.point(v) + translation_vector;
    }
}

bool repair_and_validate_mesh(Mesh mesh) {
    using namespace CGAL::Polygon_mesh_processing;

    remove_isolated_vertices(mesh);
    duplicate_non_manifold_vertices(mesh);
    stitch_borders(mesh);

    return is_valid_polygon_mesh(mesh);
}

bool read_STL(const std::string& filename, Mesh& mesh) {
    std::cout << "      Reading STL file." << std::endl;
    if (!PMP::IO::read_polygon_mesh(filename, mesh)) {
        std::cerr << "Error: Cannot read the STL file " << filename << std::endl;
        return false;
    }
    return true;
}

bool write_STL(const std::string& filename, const Mesh& mesh) {
    std::cout << "      Writting STL file." << std::endl;
    if (!CGAL::IO::write_polygon_mesh(filename, mesh, CGAL::parameters::stream_precision(17))) {
        std::cerr << "Error: Cannot write the STL file: " << filename << std::endl;
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
    Point centroid;
    
    if (!read_STL(argv[1], mesh)) {
        return EXIT_FAILURE;
    }

    CGAL::Polygon_mesh_processing::orient(mesh);
    compute_centroid(mesh, centroid);
    translate_mesh(mesh, Kernel::Vector_3(-centroid.x(), -centroid.y(), 0));

 
    shrink_mesh(mesh, std::stod(argv[3]), std::stod(argv[4]), offset_mesh); // shrink_mesh(mesh, alpha, offset)
    slice_mesh(offset_mesh, std::stod(argv[5]));
    //double offset_distance = std::stod(argv[5]);  // Negative for inward offset
    //apply_inward_offset(offset_mesh, offset_distance);

    PMP::remove_isolated_vertices(offset_mesh);
    if (!write_STL(argv[2], offset_mesh)) {
        return EXIT_FAILURE;
    }

    //// Assuming you have Boolean operations set up (e.g., via CGAL's Nef_polyhedron or Corefinement)
    //if (!PMP::corefine_and_compute_difference(offset_mesh, mesh, hollowed_mesh)) {
    //    std::cerr << "Failed to compute the hollowed mesh." << std::endl;
    //    return EXIT_FAILURE;
    //}

    //// Optionally repair the mesh
    //PMP::remove_isolated_vertices(hollowed_mesh);
    //if (!write_STL(argv[2], hollowed_mesh)) {
    //    return EXIT_FAILURE;
    //}

    std::cout << "      Hollowed mesh has been saved successfully." << std::endl;
    return EXIT_SUCCESS;
}





//#include <CGAL/IO/STL.h>
//bool read_STL(const std::string& filename, Mesh& mesh) {
//    std::cout << "      Reading STL file." << std::endl;
//    std::ifstream input(filename, std::ios::binary);
//    if (!input) {
//        std::cerr << "Error: Cannot open the STL file " << filename << std::endl;
//        return false;
//    }
//    if (!CGAL::IO::read_STL(input, mesh)) {
//        std::cerr << "Error: Cannot read the STL file " << filename << std::endl;
//        return false;
//    }
//    return true;
//}

//bool write_STL(const std::string& filename, const Mesh& mesh) {
//    std::cout << "      Writting STL file." << std::endl;
//    std::ofstream output(filename, std::ios::binary);
//    if (!output) {
//        std::cerr << "Error: Cannot open the STL file to write " << filename << std::endl;
//        return false;
//    }
//    if (!CGAL::is_valid_polygon_mesh(mesh)) {
//        std::cerr << "Error: Mesh is not valid." << std::endl;
//    }
//    output.precision(12);
//    CGAL::IO::set_binary_mode(output);
//    if (CGAL::IO::write_STL(output, mesh)) {
//        output.flush();  // Flush the stream to ensure all data is written
//        output.close();  // Close the file
//        return true;
//    }
//    else {
//        std::cerr << "Error: Cannot write the STL file: " << filename << std::endl;
//        output.close();  // Ensure file is closed even on failure
//        return false;
//    }
//    return true;
//}
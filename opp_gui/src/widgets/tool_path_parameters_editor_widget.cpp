/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "opp_gui/widgets/tool_path_parameters_editor_widget.h"

#include <QMessageBox>
#include <QProgressDialog>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <opp_area_selection/selection_artist.h>
#include <opp_path_selection/path_selection_artist.h>
#include "opp_gui/utils.h"
#include "ui_tool_path_parameters_editor.h"
#include <smooth_pose_traj/SmoothPoseTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <tool_path_planner/utilities.h>

const static std::string GENERATE_TOOLPATHS_ACTION = "generate_tool_paths";
const static std::string GENERATE_HEAT_TOOLPATHS_ACTION = "generate_heat_tool_paths";
const static std::string SMOOTH_POLYLINE_SERVICE = "polyline_smoother";

namespace opp_gui
{
ToolPathParametersEditorWidget::ToolPathParametersEditorWidget(ros::NodeHandle& nh,
                                                               const std::string& selection_world_frame,
                                                               const std::string& selection_sensor_frame,
                                                               QWidget* parent)
  : QWidget(parent), client_(GENERATE_TOOLPATHS_ACTION, false)
  , heat_client_(GENERATE_HEAT_TOOLPATHS_ACTION, false)
  , selector_(nh, selection_world_frame, selection_sensor_frame)
{
  ui_ = new Ui::ToolPathParametersEditor();
  ui_->setupUi(this);

//  ui_->double_spin_box_line_spacing->setRange(0.0, std::numeric_limits<double>::max());
//  ui_->double_spin_box_min_hole_size->setRange(0.0, std::numeric_limits<double>::max());
//  ui_->double_spin_box_point_spacing->setRange(0.0, std::numeric_limits<double>::max());
//  ui_->double_spin_box_tool_z_offset->setRange(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
//  ui_->double_spin_box_min_segment_length->setRange(0.0, std::numeric_limits<double>::max());
//  ui_->double_spin_box_intersecting_plane_height_2->setRange(0.0, std::numeric_limits<double>::max());
//  ui_->double_spin_box_raster_angle->setRange(-180., 180.);
//  ui_->spin_box_dwell_time->setRange(0, std::numeric_limits<int>::max());

  // Add values to the process type drop-down
//  ui_->combo_box_process_type->addItem("None", QVariant(opp_msgs::ProcessType::NONE));
//  ui_->combo_box_process_type->addItem("Process Paint", QVariant(opp_msgs::ProcessType::PROCESS_PAINT));
//  ui_->combo_box_process_type->addItem("Process De-paint", QVariant(opp_msgs::ProcessType::PROCESS_DEPAINT));

//  ui_->combo_box_planner_type->addItem("Plane Slicer", QVariant(noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR));
//  ui_->combo_box_planner_type->addItem("Surface Walk", QVariant(noether_msgs::ToolPathConfig::SURFACE_WALK_RASTER_GENERATOR));

  // Connect the button press to the function that calls the toolpath generation action
  connect(ui_->push_button_generate_ps, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathPlaneSlicer);
  connect(ui_->push_button_generate_sw, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathSurfaceWalker);
  connect(ui_->push_button_generate_heat, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathHeat);
  connect(ui_->push_button_generate_he, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathHalfEdge);
  connect(ui_->push_button_generate_ev, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathEigenValueEdge);
  connect(ui_->push_button_generate_manual, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::generateToolPathManual);

  connect(ui_->push_button_clear_polyline_manual, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::clearPolyline);
  connect(ui_->push_button_clear_polyline_heat, &QPushButton::clicked, this, &ToolPathParametersEditorWidget::clearPolyline);
//  connect(this, &ToolPathParametersEditorWidget::polylinePathGen, this, &ToolPathParametersEditorWidget::onPolylinePathGen);
  connect(this, &ToolPathParametersEditorWidget::QWarningBox, this, &ToolPathParametersEditorWidget::onQWarningBox);
//  connect(this, &ToolPathParametersEditorWidget::polylinePath, this, &ToolPathParametersEditorWidget::onPolylinePath);
  polyline_smooth_client_ = nh.serviceClient<smooth_pose_traj::SmoothPoseTrajectory>(SMOOTH_POLYLINE_SERVICE);
}

void ToolPathParametersEditorWidget::init(const shape_msgs::Mesh& mesh)
{
  mesh_.reset(new shape_msgs::Mesh(mesh));
  clearPolyline();
  return;
}

void ToolPathParametersEditorWidget::clearPolyline()
{
  std_srvs::Trigger srv;
  // Currently, this callback is being called directly, so it will always return true.
  bool success = selector_.clearPathPointsCb(srv.request, srv.response);
  if (!success)
  {
    emit QWarningBox("Area Selection error: could not clear polygon points");
  }
  if (!srv.response.success)
  {
    std::string msg("Area Selection error:" + srv.response.message);
    emit QWarningBox(msg.c_str());
  }

  return;
}

void ToolPathParametersEditorWidget::setToolPathConfig(const noether_msgs::ToolPathConfig& config)
{
  ui_->double_spin_box_line_spacing_ps->setValue(config.plane_slicer_generator.raster_spacing);
  ui_->double_spin_box_point_spacing_ps->setValue(config.plane_slicer_generator.point_spacing);
  ui_->double_spin_box_tool_z_offset_ps->setValue(config.plane_slicer_generator.tool_offset);
  ui_->double_spin_box_min_hole_size_ps->setValue(config.plane_slicer_generator.min_hole_size);
  ui_->double_spin_box_min_segment_length_ps->setValue(config.plane_slicer_generator.min_segment_size);
  ui_->double_spin_box_raster_angle_ps->setValue(config.plane_slicer_generator.raster_rot_offset * 180.0 / M_PI);
  ui_->double_spin_box_normal_search_radius_ps->setValue(config.plane_slicer_generator.search_radius);
  ui_->double_spin_box_raster_direction_x_ps->setValue(config.plane_slicer_generator.raster_direction.x);
  ui_->double_spin_box_raster_direction_y_ps->setValue(config.plane_slicer_generator.raster_direction.y);
  ui_->double_spin_box_raster_direction_z_ps->setValue(config.plane_slicer_generator.raster_direction.z);
  ui_->checkBox_interleave_rasters_ps->setChecked(config.plane_slicer_generator.interleave_rasters);
  ui_->checkBox_generate_extra_rasters_ps->setChecked(config.plane_slicer_generator.generate_extra_rasters);
  ui_->checkBox_raster_wrt_global_ps->setChecked(config.plane_slicer_generator.raster_wrt_global_axes);
  ui_->checkBox_smooth_rasters_ps->setChecked(config.plane_slicer_generator.smooth_rasters);
  if (config.plane_slicer_generator.raster_style == noether_msgs::ToolPathConfig::MOW_RASTER_STYLE)
      ui_->radioButton_mow_style_ps->setChecked(true);
  if (config.plane_slicer_generator.raster_style == noether_msgs::ToolPathConfig::PAINT_RASTER_STYLE)
      ui_->radioButton_paint_style_ps->setChecked(true);
  if (config.plane_slicer_generator.raster_style == noether_msgs::ToolPathConfig::READ_RASTER_STYLE)
      ui_->radioButton_read_style_ps->setChecked(true);

  ui_->double_spin_box_line_spacing_sw->setValue(config.surface_walk_generator.raster_spacing);
  ui_->double_spin_box_point_spacing_sw->setValue(config.surface_walk_generator.point_spacing);
  ui_->double_spin_box_tool_z_offset_sw->setValue(config.surface_walk_generator.tool_offset);
  ui_->double_spin_box_min_hole_size_sw->setValue(config.surface_walk_generator.min_hole_size);
  ui_->double_spin_box_min_segment_length_sw->setValue(config.plane_slicer_generator.min_segment_size);
  ui_->double_spin_box_intersecting_plane_height_sw->setValue(config.surface_walk_generator.intersection_plane_height);
  ui_->double_spin_box_raster_angle_sw->setValue(config.surface_walk_generator.raster_rot_offset * 180.0 / M_PI);
  ui_->checkBox_generate_extra_rasters_sw->setChecked(config.surface_walk_generator.generate_extra_rasters);
  ui_->double_spin_box_cut_direction_x_sw->setValue(config.surface_walk_generator.cut_direction.x);
  ui_->double_spin_box_cut_direction_y_sw->setValue(config.surface_walk_generator.cut_direction.y);
  ui_->double_spin_box_cut_direction_z_sw->setValue(config.surface_walk_generator.cut_direction.z);

  ui_->double_spin_box_point_spacing_heat->setValue(config.heat_generator.point_spacing);
  ui_->double_spin_box_line_spacing_heat->setValue(config.heat_generator.raster_spacing);
  ui_->double_spin_box_tool_z_offset_heat->setValue(config.heat_generator.tool_offset);
  ui_->double_spin_box_min_hole_size_heat->setValue(config.heat_generator.min_hole_size);
  ui_->double_spin_box_min_segment_length_heat->setValue(config.heat_generator.min_segment_size);
  ui_->double_spin_box_raster_angle_heat->setValue(config.heat_generator.raster_rot_offset * 180.0 / M_PI);
  ui_->checkBox_generate_extra_rasters_heat->setChecked(config.heat_generator.generate_extra_rasters);

  ui_->double_spin_box_min_num_pts_he->setValue(config.halfedge_generator.min_num_points);
  ui_->checkBox_normal_averaging_he->setChecked(config.halfedge_generator.normal_averaging);
  ui_->double_spin_box_normal_search_radius_he->setValue(config.halfedge_generator.normal_search_radius);
  ui_->double_spin_box_normal_influence_weight_he->setValue(config.halfedge_generator.normal_influence_weight);
  if (config.halfedge_generator.point_spacing_method == noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_NONE)
      ui_->radioButton_none_he->setChecked(true);
  if (config.halfedge_generator.point_spacing_method == noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_MIN_DISTANCE)
      ui_->radioButton_min_dist_he->setChecked(true);
  if (config.halfedge_generator.point_spacing_method == noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_EQUAL_SPACING)
      ui_->radioButton_eq_spacing_he->setChecked(true);
  if (config.halfedge_generator.point_spacing_method == noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_PARAMETRIC_SPLINE)
      ui_->radioButton_param_spline_he->setChecked(true);
  ui_->double_spin_box_point_spacing_he->setValue(config.halfedge_generator.point_dist);
  ui_->checkBox_split_by_axes_he->setChecked(config.halfedge_generator.split_by_axes);
  ui_->double_spin_box_min_pen_size_he->setValue(config.halfedge_generator.min_allowed_obstacle_width);
  ui_->double_spin_box_min_skip_amount_he->setValue(config.halfedge_generator.min_skip_amount);

  ui_->double_spin_box_octree_res_ev->setValue(config.eigen_value_generator.octree_res);
  ui_->double_spin_box_search_radius_ev->setValue(config.eigen_value_generator.search_radius);
  ui_->double_spin_box_num_threads_ev->setValue(config.eigen_value_generator.num_threads);
  ui_->double_spin_box_neighbor_tol_ev->setValue(config.eigen_value_generator.neighbor_tol);
  ui_->double_spin_box_voxel_size_ev->setValue(config.eigen_value_generator.voxel_size);
  ui_->double_spin_box_edge_cluster_min_ev->setValue(config.eigen_value_generator.edge_cluster_min);
  ui_->double_spin_box_kd_tree_ev->setValue(config.eigen_value_generator.kdtree_epsilon);
  ui_->double_spin_box_min_projection_dist_ev->setValue(config.eigen_value_generator.min_projection_dist);
  ui_->double_spin_box_max_intersecting_voxels_ev->setValue(config.eigen_value_generator.min_projection_dist);
  ui_->double_spin_box_merge_dist_ev->setValue(config.eigen_value_generator.merge_dist);
  ui_->checkBox_split_by_axes_ev->setChecked(config.eigen_value_generator.split_by_axes);

}

noether_msgs::ToolPathConfig ToolPathParametersEditorWidget::getToolPathConfig() const
{
  noether_msgs::ToolPathConfig config;

  // Create a path configuration from the line edit fields
  config.type = noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR;

  config.plane_slicer_generator.raster_spacing = ui_->double_spin_box_line_spacing_ps->value();
  config.plane_slicer_generator.point_spacing = ui_->double_spin_box_point_spacing_ps->value();
  config.plane_slicer_generator.min_segment_size = ui_->double_spin_box_min_segment_length_ps->value();
  config.plane_slicer_generator.min_hole_size = ui_->double_spin_box_min_hole_size_ps->value();
  config.plane_slicer_generator.tool_offset = ui_->double_spin_box_tool_z_offset_ps->value();
  config.plane_slicer_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_ps->value() * M_PI / 180.0;
  config.plane_slicer_generator.search_radius = ui_->double_spin_box_normal_search_radius_ps->value();
  config.plane_slicer_generator.raster_direction.x = ui_->double_spin_box_raster_direction_x_ps->value();
  config.plane_slicer_generator.raster_direction.y = ui_->double_spin_box_raster_direction_y_ps->value();
  config.plane_slicer_generator.raster_direction.z = ui_->double_spin_box_raster_direction_z_ps->value();
  config.plane_slicer_generator.interleave_rasters = ui_->checkBox_interleave_rasters_ps->isChecked();
  config.plane_slicer_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_ps->isChecked();
  config.plane_slicer_generator.raster_wrt_global_axes = ui_->checkBox_raster_wrt_global_ps->isChecked();
  config.plane_slicer_generator.smooth_rasters = ui_->checkBox_smooth_rasters_ps->isChecked();
  if (ui_->radioButton_mow_style_ps->isChecked())
    config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::MOW_RASTER_STYLE;
  if (ui_->radioButton_paint_style_ps->isChecked())
    config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::PAINT_RASTER_STYLE;
  if (ui_->radioButton_read_style_ps->isChecked())
    config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::READ_RASTER_STYLE;

  config.surface_walk_generator.point_spacing = ui_->double_spin_box_point_spacing_sw->value();
  config.surface_walk_generator.tool_offset = ui_->double_spin_box_tool_z_offset_sw->value();
  config.surface_walk_generator.raster_spacing = ui_->double_spin_box_line_spacing_sw->value();
  config.surface_walk_generator.min_hole_size = ui_->double_spin_box_min_hole_size_sw->value();
  config.surface_walk_generator.min_segment_size = ui_->double_spin_box_min_segment_length_sw->value();
  config.surface_walk_generator.intersection_plane_height = ui_->double_spin_box_intersecting_plane_height_sw->value();
  config.surface_walk_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_sw->value() * M_PI / 180.0;
  config.surface_walk_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_sw->isChecked();
  config.surface_walk_generator.cut_direction.x = ui_->double_spin_box_cut_direction_x_sw->value();
  config.surface_walk_generator.cut_direction.y = ui_->double_spin_box_cut_direction_y_sw->value();
  config.surface_walk_generator.cut_direction.z = ui_->double_spin_box_cut_direction_z_sw->value();

  config.heat_generator.point_spacing = ui_->double_spin_box_point_spacing_heat->value();
  config.heat_generator.raster_spacing = ui_->double_spin_box_line_spacing_heat->value();
  config.heat_generator.tool_offset = ui_->double_spin_box_tool_z_offset_heat->value();
  config.heat_generator.min_hole_size = ui_->double_spin_box_min_hole_size_heat->value();
  config.heat_generator.min_segment_size = ui_->double_spin_box_min_segment_length_heat->value();
  config.heat_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_heat->value() * M_PI / 180.0;
  config.heat_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_heat->isChecked();

  config.halfedge_generator.min_num_points = ui_->double_spin_box_min_num_pts_he->value();
  config.halfedge_generator.normal_averaging = ui_->checkBox_normal_averaging_he->isChecked();
  config.halfedge_generator.normal_search_radius = ui_->double_spin_box_normal_search_radius_he->value();
  config.halfedge_generator.normal_influence_weight = ui_->double_spin_box_normal_influence_weight_he->value();
  if (ui_->radioButton_none_he->isChecked())
    config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_NONE;
  if (ui_->radioButton_min_dist_he->isChecked())
    config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_MIN_DISTANCE;
  if (ui_->radioButton_eq_spacing_he->isChecked())
    config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_EQUAL_SPACING;
  if (ui_->radioButton_param_spline_he->isChecked())
    config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_PARAMETRIC_SPLINE;
  config.halfedge_generator.point_dist = ui_->double_spin_box_point_spacing_he->value();
  config.halfedge_generator.split_by_axes = ui_->checkBox_split_by_axes_he->isChecked();
  config.halfedge_generator.min_allowed_obstacle_width = ui_->double_spin_box_min_pen_size_he->value();
  config.halfedge_generator.min_skip_amount = ui_->double_spin_box_min_skip_amount_he->value();

  config.eigen_value_generator.octree_res = ui_->double_spin_box_octree_res_ev->value();
  config.eigen_value_generator.search_radius = ui_->double_spin_box_search_radius_ev->value();
  config.eigen_value_generator.num_threads = ui_->double_spin_box_num_threads_ev->value();
  config.eigen_value_generator.neighbor_tol = ui_->double_spin_box_neighbor_tol_ev->value();
  config.eigen_value_generator.voxel_size = ui_->double_spin_box_voxel_size_ev->value();
  config.eigen_value_generator.edge_cluster_min = ui_->double_spin_box_edge_cluster_min_ev->value();
  config.eigen_value_generator.kdtree_epsilon = ui_->double_spin_box_kd_tree_ev->value();
  config.eigen_value_generator.min_projection_dist = ui_->double_spin_box_min_projection_dist_ev->value();
  config.eigen_value_generator.max_intersecting_voxels = ui_->double_spin_box_max_intersecting_voxels_ev->value();
  config.eigen_value_generator.merge_dist = ui_->double_spin_box_merge_dist_ev->value();
  config.eigen_value_generator.split_by_axes = ui_->checkBox_split_by_axes_ev->isChecked();

  return config;
}

heat_msgs::HeatRasterGeneratorConfig ToolPathParametersEditorWidget::getHeatRasterGeneratorConfig() const
{
  heat_msgs::HeatRasterGeneratorConfig config;

  // Create a path configuration from the line edit fields
  config.point_spacing = ui_->double_spin_box_point_spacing_heat->value();
  config.raster_spacing = ui_->double_spin_box_line_spacing_heat->value();
  config.tool_offset = ui_->double_spin_box_tool_z_offset_heat->value();
  config.min_hole_size = ui_->double_spin_box_min_hole_size_heat->value();
  config.min_segment_size = ui_->double_spin_box_min_segment_length_heat->value();
  config.raster_rot_offset = ui_->double_spin_box_raster_angle_heat->value() * M_PI / 180.0;
  config.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_heat->isChecked();
  return config;
}

void ToolPathParametersEditorWidget::setToolPath(const opp_msgs::ToolPath& tool_path)
{
  if (!tool_path_)
  {
    tool_path_.reset(new opp_msgs::ToolPath(tool_path));
  }
  else
  {
    *tool_path_ = tool_path;
//    ui_->combo_box_process_type->setCurrentIndex(ui_->combo_box_process_type->findData(tool_path_->process_type.val));
//    ui_->spin_box_dwell_time->setValue(static_cast<int>(tool_path_->dwell_time));
  }
}

bool ToolPathParametersEditorWidget::getToolPath(opp_msgs::ToolPath& tool_path) const
{
  if (tool_path_)
  {
    tool_path = *tool_path_;
    return true;
  }
  return false;
}

void ToolPathParametersEditorWidget::generateToolPathPlaneSlicer()
{
    generateToolPath(noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR);
}

void ToolPathParametersEditorWidget::generateToolPathSurfaceWalker()
{
    generateToolPath(noether_msgs::ToolPathConfig::SURFACE_WALK_RASTER_GENERATOR);
}

void ToolPathParametersEditorWidget::generateToolPathHeat()
{
    // TODO call heat method server with mesh and pnt_indices to generate a new set of paths.
    if (!heat_client_.isServerConnected())
    {
      std::string message = "Action server on '" + GENERATE_HEAT_TOOLPATHS_ACTION + "' is not connected";
      emit QWarningBox(message);
      return;
    }

    if (!mesh_)
    {
      emit QWarningBox("Mesh has not yet been specified");
      return;
    }

    std::string error_message;
    std::vector<int> pnt_indices;
    bool success = selector_.collectPath(*mesh_, pnt_indices, error_message);
    if (!success)
    {
      std::string msg("Path Selection error: could not compute path: " + error_message);
      emit QWarningBox(msg.c_str());
    }

    // Create an action goal with only one set of sources and configs
    heat_msgs::GenerateHeatToolPathsGoal goal;
    // copy pnt_indices into goal as the sources
    heat_msgs::Source S;
    for (int i = 0; i < pnt_indices.size(); i++)
    {
      S.source_indices.push_back(pnt_indices[i]);
    }
    goal.sources.push_back(S);
    goal.path_configs.push_back(getHeatRasterGeneratorConfig());
    goal.surface_meshes.push_back(*mesh_);
    goal.proceed_on_failure = false;

    heat_client_.sendGoal(goal,
                          boost::bind(&ToolPathParametersEditorWidget::onGenerateHeatToolPathsComplete, this, _1, _2));

    progress_dialog_ = new QProgressDialog(this);
    progress_dialog_->setModal(true);
    progress_dialog_->setLabelText("Heat Path Planning Progress");
    progress_dialog_->setMinimum(0);
    progress_dialog_->setMaximum(100);

    progress_dialog_->setValue(progress_dialog_->minimum());
    progress_dialog_->show();
}

void ToolPathParametersEditorWidget::generateToolPathHalfEdge()
{
    generateToolPath(noether_msgs::ToolPathConfig::HALFEDGE_EDGE_GENERATOR);
}

void ToolPathParametersEditorWidget::generateToolPathEigenValueEdge()
{
    generateToolPath(noether_msgs::ToolPathConfig::EIGEN_VALUE_EDGE_GENERATOR);
}

void ToolPathParametersEditorWidget::generateToolPathManual()
{
  if (!mesh_)
  {
    emit QWarningBox("Mesh has not yet been specified");
    return;
  }

  std::string error_message;
  std::vector<int> pnt_indices;
  bool success = selector_.collectPath(*mesh_, pnt_indices, error_message);
  if (!success)
  {
    std::string msg("Path Selection error: could not compute path: " + error_message);
    emit QWarningBox(msg.c_str());
  }

  if (pnt_indices.size() == 0)
  {
    emit QWarningBox("No polyline points yet selected");
    return;
  }
  geometry_msgs::PoseArray output_poses = compute_pose_arrays(pnt_indices);

  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::SUCCEEDED;
  heat_msgs::GenerateHeatToolPathsResult goal_;
  goal_.success = true;
  goal_.tool_path_validities.push_back(true);
  heat_msgs::HeatToolPath tool_path;  // geometry_msgs/PoseArray[] paths
  noether_msgs::ToolPath ntool_path;  // geometry_msgs/PoseArray[] segments
  tool_path.paths.push_back(output_poses);
  goal_.tool_raster_paths.push_back(tool_path);
  auto goal = boost::make_shared<heat_msgs::GenerateHeatToolPathsResult>(goal_);

  progress_dialog_ = new QProgressDialog(this);
  progress_dialog_->setModal(true);
  progress_dialog_->setLabelText("Heat Path Planning Progress");
  progress_dialog_->setMinimum(0);
  progress_dialog_->setMaximum(100);

  progress_dialog_->setValue(progress_dialog_->minimum());
  progress_dialog_->show();

  ToolPathParametersEditorWidget::onGenerateHeatToolPathsComplete(state, goal);
}

void ToolPathParametersEditorWidget::generateToolPath(const unsigned int tp_type)
{
  if (!client_.isServerConnected())
  {
    std::string message = "Action server on '" + GENERATE_TOOLPATHS_ACTION + "' is not connected";
    emit QWarningBox(message);
    return;
  }

  if (!mesh_)
  {
    emit QWarningBox("Mesh has not yet been specified");
    return;
  }

  // Create an action goal
  noether_msgs::GenerateToolPathsGoal goal;
  auto path_config = getToolPathConfig();
  path_config.type = tp_type;
  goal.path_configs.push_back(path_config);

  goal.surface_meshes.push_back(*mesh_);
  goal.proceed_on_failure = false;

  client_.sendGoal(goal, boost::bind(&ToolPathParametersEditorWidget::onGenerateToolPathsComplete, this, _1, _2));

  progress_dialog_ = new QProgressDialog(this);
  progress_dialog_->setModal(true);
  progress_dialog_->setLabelText("Path Planning Progress");
  progress_dialog_->setMinimum(0);
  progress_dialog_->setMaximum(100);

  progress_dialog_->setValue(progress_dialog_->minimum());
  progress_dialog_->show();
}

geometry_msgs::PoseArray ToolPathParametersEditorWidget::compute_pose_arrays(const std::vector<int> pnt_indices)
{
  double point_spacing = ui_->double_spin_box_point_spacing_manual->value();
  double normal_search_radius = ui_->double_spin_box_normal_search_radius_manual->value();
  geometry_msgs::PoseArray PA;

  // compute vertex normals
  std::vector<Eigen::Vector3d> vertex_normals, face_normals;
  tool_path_planner::computeMeshNormals(*mesh_, face_normals, vertex_normals);

  // convert mesh_ to a pcl::PointCloud<pcl::PointNormal>
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  tool_path_planner::shapeMsgToPclPointXYZ(*mesh_, *mesh_cloud_ptr);

  // compute normals of mesh using Moving Least Squares
  auto mesh_normals_ptr = boost::make_shared<pcl::PointCloud<pcl::PointNormal> >(
      tool_path_planner::computeMLSMeshNormals(mesh_cloud_ptr, normal_search_radius));

  // align the mls normals to the vertex normals
  if (!tool_path_planner::alignToVertexNormals(*mesh_normals_ptr, vertex_normals))
  {
    ROS_ERROR("alignToVertexNormals failed");
  }

  // generate equally spaced points
  pcl::PointCloud<pcl::PointNormal> equal_spaced_points;
  if (!tool_path_planner::applyEqualDistance(pnt_indices, *mesh_cloud_ptr, equal_spaced_points, point_spacing))
  {
    ROS_ERROR("could not find equal spaced points");
  }
  else
  {
    tool_path_planner::insertPointNormals(mesh_normals_ptr, equal_spaced_points);
    tool_path_planner::ToolPathSegment segment;
    if (!tool_path_planner::createToolPathSegment(equal_spaced_points, {}, segment))
    {
      ROS_ERROR("failed to create tool path segment from equally spaced points");
    }
    else
    {
      for (Eigen::Isometry3d pose : segment)
      {
        geometry_msgs::Pose gm_pose;
        Eigen::Quaterniond r(pose.rotation());
        gm_pose.position.x = pose.translation().x();
        gm_pose.position.y = pose.translation().y();
        gm_pose.position.z = pose.translation().z();
        gm_pose.orientation.x = r.x();
        gm_pose.orientation.y = r.y();
        gm_pose.orientation.z = r.z();
        gm_pose.orientation.w = r.w();
        PA.poses.push_back(gm_pose);
      }
    }
  }
  return (PA);
}

//void ToolPathParametersEditorWidget::onPolylinePath(const std::vector<int> pnt_indices)
//{
//  if (!mesh_)
//  {
//    emit QWarningBox("Mesh has not yet been specified");
//    return;
//  }
//  if (pnt_indices.size() == 0)
//  {
//    emit QWarningBox("No polyline points yet selected");
//    return;
//  }
//  geometry_msgs::PoseArray output_poses = compute_pose_arrays(pnt_indices);

//  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::SUCCEEDED;
//  heat_msgs::GenerateHeatToolPathsResult goal_;
//  goal_.success = true;
//  goal_.tool_path_validities.push_back(true);
//  heat_msgs::HeatToolPath tool_path;  // geometry_msgs/PoseArray[] paths
//  noether_msgs::ToolPath ntool_path;  // geometry_msgs/PoseArray[] segments
//  tool_path.paths.push_back(output_poses);
//  goal_.tool_raster_paths.push_back(tool_path);
//  auto goal = boost::make_shared<heat_msgs::GenerateHeatToolPathsResult>(goal_);

//  progress_dialog_ = new QProgressDialog(this);
//  progress_dialog_->setModal(true);
//  progress_dialog_->setLabelText("Heat Path Planning Progress");
//  progress_dialog_->setMinimum(0);
//  progress_dialog_->setMaximum(100);

//  progress_dialog_->setValue(progress_dialog_->minimum());
//  progress_dialog_->show();

//  ToolPathParametersEditorWidget::onGenerateHeatToolPathsComplete(state, goal);
//}

//void ToolPathParametersEditorWidget::onPolylinePathGen(const std::vector<int> pnt_indices)
//{
//  // TODO call heat method server with mesh and pnt_indices to generate a new set of paths.
//  if (!heat_client_.isServerConnected())
//  {
//    std::string message = "Action server on '" + GENERATE_HEAT_TOOLPATHS_ACTION + "' is not connected";
//    emit QWarningBox(message);
//    return;
//  }

//  if (!mesh_)
//  {
//    emit QWarningBox("Mesh has not yet been specified");
//    return;
//  }

//  // Create an action goal with only one set of sources and configs
//  heat_msgs::GenerateHeatToolPathsGoal goal;
//  // copy pnt_indices into goal as the sources
//  heat_msgs::Source S;
//  for (int i = 0; i < pnt_indices.size(); i++)
//  {
//    S.source_indices.push_back(pnt_indices[i]);
//  }
//  goal.sources.push_back(S);
//  goal.path_configs.push_back(getHeatRasterGeneratorConfig());
//  goal.surface_meshes.push_back(*mesh_);
//  goal.proceed_on_failure = false;

//  heat_client_.sendGoal(goal,
//                        boost::bind(&ToolPathParametersEditorWidget::onGenerateHeatToolPathsComplete, this, _1, _2));

//  progress_dialog_ = new QProgressDialog(this);
//  progress_dialog_->setModal(true);
//  progress_dialog_->setLabelText("Heat Path Planning Progress");
//  progress_dialog_->setMinimum(0);
//  progress_dialog_->setMaximum(100);

//  progress_dialog_->setValue(progress_dialog_->minimum());
//  progress_dialog_->show();
//}

void ToolPathParametersEditorWidget::onGenerateToolPathsComplete(
    const actionlib::SimpleClientGoalState& state,
    const noether_msgs::GenerateToolPathsResultConstPtr& res)
{
  for (int i = progress_dialog_->minimum(); i < progress_dialog_->maximum(); ++i)
  {
    progress_dialog_->setValue(i);
    ros::Duration(0.01).sleep();
  }
  progress_dialog_->hide();

  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::string message = "Action '" + GENERATE_TOOLPATHS_ACTION + "' failed to succeed";
    emit QWarningBox(message);
  }
  else
  {
    if (!res->success || !res->tool_path_validities[0])
    {
      emit QWarningBox("Tool path generation failed");
    }
    else if (res->tool_paths[0].paths.size() < 1 || res->tool_paths[0].paths[0].segments.size() < 1)
    {
      emit QWarningBox("Tool path generation succeeded, but not paths returned");
    }
    else
    {
      ROS_INFO_STREAM("Successfully generated tool path returned from action");
      opp_msgs::ToolPath tp;
      tp.header.stamp = ros::Time::now();
//      tp.process_type.val = qvariant_cast<opp_msgs::ProcessType::_val_type>(ui_->combo_box_process_type->currentData());
      tp.process_type.val = noether_msgs::ToolPathConfig::PLANE_SLICER_RASTER_GENERATOR;
      std::cout << "# tool_paths[0].paths " << res->tool_paths[0].paths.size() << std::endl;
      for (size_t i = 0; i < res->tool_paths[0].paths.size(); i++)
      {
          std::cout << "# tool_paths[0].paths[" << i << "].segments " << res->tool_paths[0].paths[i].segments.size() << std::endl;
        for (size_t j = 0; j < res->tool_paths[0].paths[i].segments.size(); j++)
        {
          tp.paths.push_back(res->tool_paths[0].paths[i].segments[j]);
        }
      }

      tp.params.config.plane_slicer_generator.raster_spacing = ui_->double_spin_box_line_spacing_ps->value();
      tp.params.config.plane_slicer_generator.point_spacing = ui_->double_spin_box_point_spacing_ps->value();
      tp.params.config.plane_slicer_generator.min_segment_size = ui_->double_spin_box_min_segment_length_ps->value();
      tp.params.config.plane_slicer_generator.min_hole_size = ui_->double_spin_box_min_hole_size_ps->value();
      tp.params.config.plane_slicer_generator.tool_offset = ui_->double_spin_box_tool_z_offset_ps->value();
      tp.params.config.plane_slicer_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_ps->value() * M_PI / 180.0;
      tp.params.config.plane_slicer_generator.search_radius = ui_->double_spin_box_normal_search_radius_ps->value();
      tp.params.config.plane_slicer_generator.raster_direction.x = ui_->double_spin_box_raster_direction_x_ps->value();
      tp.params.config.plane_slicer_generator.raster_direction.y = ui_->double_spin_box_raster_direction_y_ps->value();
      tp.params.config.plane_slicer_generator.raster_direction.z = ui_->double_spin_box_raster_direction_z_ps->value();
      tp.params.config.plane_slicer_generator.interleave_rasters = ui_->checkBox_interleave_rasters_ps->isChecked();
      tp.params.config.plane_slicer_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_ps->isChecked();
      tp.params.config.plane_slicer_generator.raster_wrt_global_axes = ui_->checkBox_raster_wrt_global_ps->isChecked();
      tp.params.config.plane_slicer_generator.smooth_rasters = ui_->checkBox_smooth_rasters_ps->isChecked();
      if (ui_->radioButton_mow_style_ps->isChecked())
        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::MOW_RASTER_STYLE;
      if (ui_->radioButton_paint_style_ps->isChecked())
        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::PAINT_RASTER_STYLE;
      if (ui_->radioButton_read_style_ps->isChecked())
        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::READ_RASTER_STYLE;

      tp.params.config.surface_walk_generator.point_spacing = ui_->double_spin_box_point_spacing_sw->value();
      tp.params.config.surface_walk_generator.tool_offset = ui_->double_spin_box_tool_z_offset_sw->value();
      tp.params.config.surface_walk_generator.raster_spacing = ui_->double_spin_box_line_spacing_sw->value();
      tp.params.config.surface_walk_generator.min_hole_size = ui_->double_spin_box_min_hole_size_sw->value();
      tp.params.config.surface_walk_generator.min_segment_size = ui_->double_spin_box_min_segment_length_sw->value();
      tp.params.config.surface_walk_generator.intersection_plane_height = ui_->double_spin_box_intersecting_plane_height_sw->value();
      tp.params.config.surface_walk_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_sw->value() * M_PI / 180.0;
      tp.params.config.surface_walk_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_sw->isChecked();
      tp.params.config.surface_walk_generator.cut_direction.x = ui_->double_spin_box_cut_direction_x_sw->value();
      tp.params.config.surface_walk_generator.cut_direction.y = ui_->double_spin_box_cut_direction_y_sw->value();
      tp.params.config.surface_walk_generator.cut_direction.z = ui_->double_spin_box_cut_direction_z_sw->value();

      tp.params.config.heat_generator.point_spacing = ui_->double_spin_box_point_spacing_heat->value();
      tp.params.config.heat_generator.raster_spacing = ui_->double_spin_box_line_spacing_heat->value();
      tp.params.config.heat_generator.tool_offset = ui_->double_spin_box_tool_z_offset_heat->value();
      tp.params.config.heat_generator.min_hole_size = ui_->double_spin_box_min_hole_size_heat->value();
      tp.params.config.heat_generator.min_segment_size = ui_->double_spin_box_min_segment_length_heat->value();
      tp.params.config.heat_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_heat->value() * M_PI / 180.0;
      tp.params.config.heat_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_heat->isChecked();

      tp.params.config.halfedge_generator.min_num_points = ui_->double_spin_box_min_num_pts_he->value();
      tp.params.config.halfedge_generator.normal_averaging = ui_->checkBox_normal_averaging_he->isChecked();
      tp.params.config.halfedge_generator.normal_search_radius = ui_->double_spin_box_normal_search_radius_he->value();
      tp.params.config.halfedge_generator.normal_influence_weight = ui_->double_spin_box_normal_influence_weight_he->value();
      if (ui_->radioButton_none_he->isChecked())
        tp.params.config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_NONE;
      if (ui_->radioButton_min_dist_he->isChecked())
        tp.params.config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_MIN_DISTANCE;
      if (ui_->radioButton_eq_spacing_he->isChecked())
        tp.params.config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_EQUAL_SPACING;
      if (ui_->radioButton_param_spline_he->isChecked())
        tp.params.config.halfedge_generator.point_spacing_method = noether_msgs::HalfedgeEdgeGeneratorConfig::POINT_SPACING_METHOD_PARAMETRIC_SPLINE;
      tp.params.config.halfedge_generator.point_dist = ui_->double_spin_box_point_spacing_he->value();
      tp.params.config.halfedge_generator.split_by_axes = ui_->checkBox_split_by_axes_he->isChecked();

      tp.params.config.eigen_value_generator.octree_res = ui_->double_spin_box_octree_res_ev->value();
      tp.params.config.eigen_value_generator.search_radius = ui_->double_spin_box_search_radius_ev->value();
      tp.params.config.eigen_value_generator.num_threads = ui_->double_spin_box_num_threads_ev->value();
      tp.params.config.eigen_value_generator.neighbor_tol = ui_->double_spin_box_neighbor_tol_ev->value();
      tp.params.config.eigen_value_generator.voxel_size = ui_->double_spin_box_voxel_size_ev->value();
      tp.params.config.eigen_value_generator.edge_cluster_min = ui_->double_spin_box_edge_cluster_min_ev->value();
      tp.params.config.eigen_value_generator.kdtree_epsilon = ui_->double_spin_box_kd_tree_ev->value();
      tp.params.config.eigen_value_generator.min_projection_dist = ui_->double_spin_box_min_projection_dist_ev->value();
      tp.params.config.eigen_value_generator.max_intersecting_voxels = ui_->double_spin_box_max_intersecting_voxels_ev->value();
      tp.params.config.eigen_value_generator.merge_dist = ui_->double_spin_box_merge_dist_ev->value();
      tp.params.config.eigen_value_generator.split_by_axes = ui_->checkBox_split_by_axes_ev->isChecked();

//      tp.params.config.surface_walk_generator.point_spacing = ui_->double_spin_box_point_spacing->value();
//      tp.params.config.surface_walk_generator.tool_offset = ui_->double_spin_box_tool_z_offset->value();
//      tp.params.config.surface_walk_generator.raster_spacing = ui_->double_spin_box_line_spacing->value();
//      tp.params.config.surface_walk_generator.raster_rot_offset =
//          ui_->double_spin_box_raster_angle->value() * M_PI / 180.0;
//      tp.params.config.surface_walk_generator.min_hole_size = ui_->double_spin_box_min_hole_size->value();
//      tp.params.config.surface_walk_generator.min_segment_size = ui_->double_spin_box_min_segment_length->value();
//      tp.params.config.surface_walk_generator.generate_extra_rasters =
//          ui_->checkBox_generate_extra_rasters->isChecked();
//      tp.params.config.surface_walk_generator.intersection_plane_height =
//          ui_->double_spin_box_intersecting_plane_height_2->value();

//      tp.params.config.plane_slicer_generator.raster_spacing = ui_->double_spin_box_line_spacing->value();
//      tp.params.config.plane_slicer_generator.point_spacing = ui_->double_spin_box_point_spacing->value();
//      tp.params.config.plane_slicer_generator.tool_offset = ui_->double_spin_box_tool_z_offset->value();
//      tp.params.config.plane_slicer_generator.min_hole_size = ui_->double_spin_box_min_hole_size->value();
//      tp.params.config.plane_slicer_generator.min_segment_size = ui_->double_spin_box_min_segment_length->value();
//      tp.params.config.plane_slicer_generator.raster_rot_offset =
//          ui_->double_spin_box_raster_angle->value() * M_PI / 180.0;
//      tp.params.config.plane_slicer_generator.search_radius = ui_->double_spin_box_normal_search_radius->value();
//      tp.params.config.plane_slicer_generator.raster_direction.x = ui_->double_spin_box_raster_direction_x->value();
//      tp.params.config.plane_slicer_generator.raster_direction.y = ui_->double_spin_box_raster_direction_y->value();
//      tp.params.config.plane_slicer_generator.raster_direction.z = ui_->double_spin_box_raster_direction_z->value();
//      tp.params.config.plane_slicer_generator.interleave_rasters = ui_->checkBox_interleave_rasters->isChecked();
//      tp.params.config.plane_slicer_generator.smooth_rasters = ui_->checkBox_smooth_rasters->isChecked();
//      tp.params.config.plane_slicer_generator.generate_extra_rasters =
//          ui_->checkBox_generate_extra_rasters->isChecked();
//      tp.params.config.plane_slicer_generator.raster_wrt_global_axes = ui_->checkBox_raster_wrt_global->isChecked();
//      if (ui_->radioButton_mow_style->isChecked())
//        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::MOW_RASTER_STYLE;
//      if (ui_->radioButton_paint_style->isChecked())
//        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::PAINT_RASTER_STYLE;
//      if (ui_->radioButton_read_style->isChecked())
//        tp.params.config.plane_slicer_generator.raster_style = noether_msgs::ToolPathConfig::READ_RASTER_STYLE;

      // Create the tool path offset transform
      // 1. Add z offset
      // 2. Rotate 180 degrees about X
      Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();
      tool_offset.rotate(
          Eigen::AngleAxisd(ui_->double_spin_box_tool_pitch_ps->value() * M_PI / 180.0, Eigen::Vector3d::UnitY()));
      tool_offset.translate(Eigen::Vector3d(0.0, 0.0, ui_->double_spin_box_tool_z_offset_ps->value()));
      tool_offset.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      tf::poseEigenToMsg(tool_offset, tp.tool_offset);

      // Save the toolpath
      tool_path_.reset(new opp_msgs::ToolPath(tp));

      emit dataChanged();
    }
  }
}

void ToolPathParametersEditorWidget::onQWarningBox(std::string warn_string)
{
  QMessageBox::warning(this, "Tool Path Planning Warning", QString(warn_string.c_str()));
}

void ToolPathParametersEditorWidget::onGenerateHeatToolPathsComplete(
    const actionlib::SimpleClientGoalState& state,
    const heat_msgs::GenerateHeatToolPathsResultConstPtr& res)
{
  for (int i = progress_dialog_->minimum(); i < progress_dialog_->maximum(); ++i)
  {
    progress_dialog_->setValue(i);
    ros::Duration(0.01).sleep();
  }
  progress_dialog_->hide();

  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::string message = "Action '" + GENERATE_HEAT_TOOLPATHS_ACTION + "' failed to succeed";
    emit QWarningBox(message);
  }
  else
  {
    if (!res->success || !res->tool_path_validities[0])
    {
      emit QWarningBox(std::string("Heat tool path generation failed"));
    }
    else
    {
      ROS_INFO_STREAM("Successfully generated heat tool path");

      opp_msgs::ToolPath tp;
      tp.header.stamp = ros::Time::now();
//      tp.process_type.val = qvariant_cast<opp_msgs::ProcessType::_val_type>(ui_->combo_box_process_type->currentData());
      tp.paths = res->tool_raster_paths[0].paths;
      tp.params.config.type = 2;
      tp.params.config.heat_generator.point_spacing = ui_->double_spin_box_point_spacing_heat->value();
      tp.params.config.heat_generator.raster_spacing = ui_->double_spin_box_line_spacing_heat->value();
      tp.params.config.heat_generator.tool_offset = ui_->double_spin_box_tool_z_offset_heat->value();
      tp.params.config.heat_generator.min_hole_size = ui_->double_spin_box_min_hole_size_heat->value();
      tp.params.config.heat_generator.min_segment_size = ui_->double_spin_box_min_segment_length_heat->value();
      tp.params.config.heat_generator.raster_rot_offset = ui_->double_spin_box_raster_angle_heat->value() * M_PI / 180.0;
      tp.params.config.heat_generator.generate_extra_rasters = ui_->checkBox_generate_extra_rasters_heat->isChecked();

      // Create the tool path offset transform
      // 1. Add z offset
      // 2. Rotate 180 degrees about X
      Eigen::Isometry3d tool_offset = Eigen::Isometry3d::Identity();
      tool_offset.rotate(
          Eigen::AngleAxisd(ui_->double_spin_box_tool_pitch_heat->value() * M_PI / 180.0, Eigen::Vector3d::UnitY()));
      tool_offset.translate(Eigen::Vector3d(0.0, 0.0, ui_->double_spin_box_tool_z_offset_heat->value()));
      tool_offset.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      tf::poseEigenToMsg(tool_offset, tp.tool_offset);

      // Save the toolpath
      tool_path_.reset(new opp_msgs::ToolPath(tp));

      emit dataChanged();
    }
  }
}

}  // namespace opp_gui

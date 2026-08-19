add_executable(test_visual_feature_track_manager
    "${CMAKE_CURRENT_LIST_DIR}/test_visual_feature_track_manager.cpp"
)
target_link_libraries(test_visual_feature_track_manager PRIVATE
    drone_test_support
    Eigen3::Eigen
)
drone_register_gtest(test_visual_feature_track_manager "unit;navigation;visual-feature-tracking")

add_executable(test_visual_feature_track_ingest
    "${CMAKE_CURRENT_LIST_DIR}/test_visual_feature_track_ingest.cpp"
)
target_link_libraries(test_visual_feature_track_ingest PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_visual_feature_track_ingest "unit;navigation;shadow-only;visual-feature-tracking")

add_executable(visual_feature_track_ingest_replay
    "${CMAKE_CURRENT_LIST_DIR}/visual_feature_track_ingest_replay.cpp"
)
target_link_libraries(visual_feature_track_ingest_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(visual_feature_track_ingest_replay)
drone_enable_coverage(visual_feature_track_ingest_replay)
drone_enable_sanitizers(visual_feature_track_ingest_replay)
drone_set_target_output_dirs(visual_feature_track_ingest_replay "tests" "tests" "tests")
add_test(NAME visual_feature_track_ingest_replay COMMAND $<TARGET_FILE:visual_feature_track_ingest_replay>)
set_tests_properties(visual_feature_track_ingest_replay PROPERTIES
    LABELS "integration;navigation;replay;shadow-only;visual-feature-tracking")

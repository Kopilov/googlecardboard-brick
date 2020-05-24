/*
 * Forked from Google sample
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cardboard_brick_app.h"

#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <android/log.h>

#include <array>
#include <cmath>
#include <fstream>

namespace ndk_cardboard {

namespace {

// The objects are about 1 meter in radius, so the min/max target distance are
// set so that the objects are always within the room (which is about 5 meters
// across) and the reticle is always closer than any objects.
constexpr float kMinTargetDistance = 2.5f;
constexpr float kMaxTargetDistance = 3.5f;
constexpr float kMinTargetHeight = 0.5f;
constexpr float kMaxTargetHeight = kMinTargetHeight + 3.0f;

constexpr float kDefaultFloorHeight = -1.7f;

constexpr uint64_t kPredictionTimeWithoutVsyncNanos = 50000000;

// Angle threshold for determining whether the controller is pointing at the
// object.
constexpr float kAngleLimit = 0.2f;

// Number of different possible targets
constexpr int kTargetMeshCount = 3;

// Simple shaders to render .obj files without any lighting.
constexpr const char* kObjVertexShaders =
    R"glsl(
    uniform mat4 u_MVP;
    attribute vec4 a_Position;
    attribute vec2 a_UV;
    varying vec2 v_UV;

    void main() {
      v_UV = a_UV;
      gl_Position = u_MVP * a_Position;
    })glsl";

constexpr const char* kObjFragmentShaders =
    R"glsl(
    precision mediump float;
    varying vec2 v_UV;
    uniform sampler2D u_Texture;

    void main() {
      // The y coordinate of this sample's textures is reversed compared to
      // what OpenGL expects, so we invert the y coordinate.
      gl_FragColor = texture2D(u_Texture, vec2(v_UV.x, 1.0 - v_UV.y));
    })glsl";

}  // anonymous namespace

CardboardBrickApp::CardboardBrickApp(JavaVM* vm, jobject obj, jobject asset_mgr_obj)
    : head_tracker_(nullptr),
      lens_distortion_(nullptr),
      distortion_renderer_(nullptr),
      screen_params_changed_(false),
      device_params_changed_(false),
      screen_width_(0),
      screen_height_(0),
      depthRenderBuffer_(0),
      framebuffer_(0),
      texture_(0),
      obj_program_(0),
      obj_position_param_(0),
      obj_uv_param_(0),
      obj_modelview_projection_param_(0),
      target_object_meshes_(kTargetMeshCount),
      target_object_not_selected_textures_(kTargetMeshCount),
      target_object_selected_textures_(kTargetMeshCount),
      cur_target_object_(RandomUniformInt(kTargetMeshCount)
  ) {
  JNIEnv* env;
  vm->GetEnv((void**)&env, JNI_VERSION_1_6);
  java_asset_mgr_ = env->NewGlobalRef(asset_mgr_obj);
  asset_mgr_ = AAssetManager_fromJava(env, asset_mgr_obj);
  Cardboard_initializeAndroid(vm, obj);
  head_tracker_ = CardboardHeadTracker_create();
  model = nullptr;
  shader = nullptr;
}

CardboardBrickApp::~CardboardBrickApp() {
  if (shader != nullptr) delete shader;
  if (model != nullptr) delete model;
  CardboardHeadTracker_destroy(head_tracker_);
  CardboardLensDistortion_destroy(lens_distortion_);
  CardboardDistortionRenderer_destroy(distortion_renderer_);
}

void CardboardBrickApp::OnSurfaceCreated(JNIEnv* env, jobject activityObject) {
  const int obj_vertex_shader =
      LoadGLShader(GL_VERTEX_SHADER, kObjVertexShaders);
  const int obj_fragment_shader =
      LoadGLShader(GL_FRAGMENT_SHADER, kObjFragmentShaders);

  obj_program_ = glCreateProgram();
  glAttachShader(obj_program_, obj_vertex_shader);
  glAttachShader(obj_program_, obj_fragment_shader);
  glLinkProgram(obj_program_);
  glUseProgram(obj_program_);

  CHECKGLERROR("Obj program");

  obj_position_param_ = glGetAttribLocation(obj_program_, "a_Position");
  obj_uv_param_ = glGetAttribLocation(obj_program_, "a_UV");
  obj_modelview_projection_param_ = glGetUniformLocation(obj_program_, "u_MVP");

  CHECKGLERROR("Obj program params");

  CARDBOARD_CHECK(room_.Initialize(env, asset_mgr_, "CubeRoom.obj",
                                   obj_position_param_, obj_uv_param_));
  CARDBOARD_CHECK(
      room_tex_.Initialize(env, java_asset_mgr_, "CubeRoom_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_meshes_[0].Initialize(
      env, asset_mgr_, "Icosahedron.obj", obj_position_param_, obj_uv_param_));
  CARDBOARD_CHECK(target_object_not_selected_textures_[0].Initialize(
      env, java_asset_mgr_, "Icosahedron_Blue_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_selected_textures_[0].Initialize(
      env, java_asset_mgr_, "Icosahedron_Pink_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_meshes_[1].Initialize(
      env, asset_mgr_, "QuadSphere.obj", obj_position_param_, obj_uv_param_));
  CARDBOARD_CHECK(target_object_not_selected_textures_[1].Initialize(
      env, java_asset_mgr_, "QuadSphere_Blue_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_selected_textures_[1].Initialize(
      env, java_asset_mgr_, "QuadSphere_Pink_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_meshes_[2].Initialize(
      env, asset_mgr_, "TriSphere.obj", obj_position_param_, obj_uv_param_));
  CARDBOARD_CHECK(target_object_not_selected_textures_[2].Initialize(
      env, java_asset_mgr_, "TriSphere_Blue_BakedDiffuse.png"));
  CARDBOARD_CHECK(target_object_selected_textures_[2].Initialize(
      env, java_asset_mgr_, "TriSphere_Pink_BakedDiffuse.png"));

  // Target object first appears directly in front of user.
  model_target_ = GetTranslationMatrix({0.0f, 1.5f, kMinTargetDistance});

  model = new Model(getAssetLocation(env, activityObject, "brick.obj"));
  shader = new Shader(
      getAssetLocation(env, activityObject, "vertexShader.gl").c_str(),
      getAssetLocation(env, activityObject, "fragmentShader.gl").c_str()
  );
  CHECKGLERROR("OnSurfaceCreated");
}

void CardboardBrickApp::SetScreenParams(int width, int height) {
  screen_width_ = width;
  screen_height_ = height;
  screen_params_changed_ = true;
}

void CardboardBrickApp::OnDrawFrame(long timestamp) {
  if (!UpdateDeviceParams()) {
    return;
  }

  // Update Head Pose.
  head_view_ = GetPose();

  // Incorporate the floor height into the head_view
//  head_view_ =
//      head_view_ * GetTranslationMatrix({0.0f, kDefaultFloorHeight, 0.0f});

  // Bind buffer
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glDisable(GL_SCISSOR_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw eyes views
  for (int eye = 0; eye < 2; ++eye) {
    glViewport(eye == kLeft ? 0 : screen_width_ / 2, 0, screen_width_ / 2,
               screen_height_);

    Matrix4x4 eye_matrix = GetMatrixFromGlArray(eye_matrices_[eye]);
    Matrix4x4 eye_view = eye_matrix * head_view_;

    Matrix4x4 projection_matrix =
        GetMatrixFromGlArray(projection_matrices_[eye]);
    Matrix4x4 modelview_target = eye_view * model_target_;
    modelview_projection_target_ = projection_matrix * modelview_target;
    modelview_projection_room_ = projection_matrix * eye_view;

    // Draw room and target
//    DrawWorld();
    DrawBrick(timestamp, glm::make_mat4(eye_view.ToGlArray().data()), glm::make_mat4(projection_matrices_[eye]));
//    DrawBrick(timestamp, glm::make_mat4(head_view_.ToGlArray().data()), glm::make_mat4(projection_matrices_[eye]));
  }

  // Render
  CardboardDestortionRenderer_renderEyeToDisplay(
      distortion_renderer_, 0, screen_width_, screen_height_,
      &left_eye_texture_description_, &right_eye_texture_description_);

  CHECKGLERROR("onDrawFrame");
}

void CardboardBrickApp::OnTriggerEvent() {
  if (IsPointingAtTarget()) {
    HideTarget();
  }
}

void CardboardBrickApp::OnPause() { CardboardHeadTracker_pause(head_tracker_); }

void CardboardBrickApp::OnResume() {
  CardboardHeadTracker_resume(head_tracker_);

  // Parameters may have changed.
  device_params_changed_ = true;

  // Check for device parameters existence in external storage. If they're
  // missing, we must scan a Cardboard QR code and save the obtained parameters.
  uint8_t* buffer;
  int size;
  CardboardQrCode_getSavedDeviceParams(&buffer, &size);
  if (size == 0) {
    SwitchViewer();
  }
  CardboardQrCode_destroy(buffer);
}

void CardboardBrickApp::SwitchViewer() {
  CardboardQrCode_scanQrCodeAndSaveDeviceParams();
}

bool CardboardBrickApp::UpdateDeviceParams() {
  // Checks if screen or device parameters changed
  if (!screen_params_changed_ && !device_params_changed_) {
    return true;
  }

  // Get saved device parameters
  uint8_t* buffer;
  int size;
  CardboardQrCode_getSavedDeviceParams(&buffer, &size);

  // If there are no parameters saved yet, returns false.
  if (size == 0) {
    return false;
  }

  CardboardLensDistortion_destroy(lens_distortion_);
  lens_distortion_ = CardboardLensDistortion_create(
      buffer, size, screen_width_, screen_height_);

  CardboardQrCode_destroy(buffer);

  GlSetup();

  CardboardDistortionRenderer_destroy(distortion_renderer_);
  distortion_renderer_ = CardboardDistortionRenderer_create();

  CardboardMesh left_mesh;
  CardboardMesh right_mesh;
  CardboardLensDistortion_getDistortionMesh(lens_distortion_, kLeft,
                                            &left_mesh);
  CardboardLensDistortion_getDistortionMesh(lens_distortion_, kRight,
                                            &right_mesh);

  CardboardDistortionRenderer_setMesh(distortion_renderer_, &left_mesh, kLeft);
  CardboardDistortionRenderer_setMesh(distortion_renderer_, &right_mesh,
                                      kRight);

  // Get eye matrices
  CardboardLensDistortion_getEyeMatrices(
      lens_distortion_, projection_matrices_[0], eye_matrices_[0], kLeft);
  CardboardLensDistortion_getEyeMatrices(
      lens_distortion_, projection_matrices_[1], eye_matrices_[1], kRight);

  screen_params_changed_ = false;
  device_params_changed_ = false;

  CHECKGLERROR("UpdateDeviceParams");

  return true;
}

void CardboardBrickApp::GlSetup() {
  LOGD("GL SETUP");

  if (framebuffer_ != 0) {
    GlTeardown();
  }

  // Create render texture.
  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, screen_width_, screen_height_, 0,
               GL_RGB, GL_UNSIGNED_BYTE, 0);

  left_eye_texture_description_.texture = texture_;
  left_eye_texture_description_.layer = 0;
  left_eye_texture_description_.left_u = 0;
  left_eye_texture_description_.right_u = 0.5;
  left_eye_texture_description_.top_v = 1;
  left_eye_texture_description_.bottom_v = 0;

  right_eye_texture_description_.texture = texture_;
  right_eye_texture_description_.layer = 0;
  right_eye_texture_description_.left_u = 0.5;
  right_eye_texture_description_.right_u = 1;
  right_eye_texture_description_.top_v = 1;
  right_eye_texture_description_.bottom_v = 0;

  // Generate depth buffer to perform depth test.
  glGenRenderbuffers(1, &depthRenderBuffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, screen_width_,
                        screen_height_);
  CHECKGLERROR("Create Render buffer");

  // Create render target.
  glGenFramebuffers(1, &framebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texture_, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, depthRenderBuffer_);

  CHECKGLERROR("GlSetup");
}

void CardboardBrickApp::GlTeardown() {
  if (framebuffer_ == 0) {
    return;
  }
  glDeleteRenderbuffers(1, &depthRenderBuffer_);
  depthRenderBuffer_ = 0;
  glDeleteFramebuffers(1, &framebuffer_);
  framebuffer_ = 0;
  glDeleteTextures(1, &texture_);
  texture_ = 0;

  CHECKGLERROR("GlTeardown");
}

Matrix4x4 CardboardBrickApp::GetPose() {
  std::array<float, 4> out_orientation;
  std::array<float, 3> out_position;
  long monotonic_time_nano = GetMonotonicTimeNano();
  monotonic_time_nano += kPredictionTimeWithoutVsyncNanos;
  CardboardHeadTracker_getPose(head_tracker_, monotonic_time_nano,
                               &out_position[0], &out_orientation[0]);
  return GetTranslationMatrix(out_position) *
         Quatf::FromXYZW(&out_orientation[0]).ToMatrix();
}

void CardboardBrickApp::DrawWorld() {
  DrawRoom();
  DrawTarget();
}

void CardboardBrickApp::DrawTarget() {
  glUseProgram(obj_program_);

  std::array<float, 16> target_array = modelview_projection_target_.ToGlArray();
  glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                     target_array.data());

  if (IsPointingAtTarget()) {
    target_object_selected_textures_[cur_target_object_].Bind();
  } else {
    target_object_not_selected_textures_[cur_target_object_].Bind();
  }
  target_object_meshes_[cur_target_object_].Draw();

  CHECKGLERROR("DrawTarget");
}

void CardboardBrickApp::DrawRoom() {
  glUseProgram(obj_program_);

  std::array<float, 16> room_array = modelview_projection_room_.ToGlArray();
  glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                     room_array.data());

  room_tex_.Bind();
  room_.Draw();

  CHECKGLERROR("DrawRoom");
}

void CardboardBrickApp::DrawBrick(long timestamp, glm::mat4 v, glm::mat4 p) {
  float ratio = 1.0;
  glm::mat4 pL = glm::perspective(1.8f, ratio, 0.001f, 60.0f);

  glm::mat4 m = glm::mat4(1.0f);
  m = glm::translate(m, glm::vec3(-1.0f, 0.0f, -5.0f));
  float timeR = ((float)(timestamp % 3000000000L)) / 1000.0f;
  m = glm::rotate(m, timeR * 2, glm::vec3(1.0f, 0.0f, 0.0f));
  m = glm::rotate(m, timeR / 2, glm::vec3(0.0f, 1.0f, 0.0f));

  glm::mat4 vL = glm::mat4(1.0f);

  shader->use();
  shader->setMat4("model", glm::value_ptr(m));
  shader->setMat4("view", glm::value_ptr(v));
  shader->setMat4("projection", glm::value_ptr(p));
  shader->setVec3("lightPos", 2.0, 2.0, -3.0);

  shader->setVec3("light.ambient",  0.2f, 0.2f, 0.2f);
  shader->setVec3("light.diffuse",  1.0f, 1.0f, 1.0f);
  shader->setVec3("light.specular", 1.0f, 1.0f, 1.0f);

  model->Draw(*shader);
  CHECKGLERROR("DrawBrick");
}

void CardboardBrickApp::HideTarget() {
  cur_target_object_ = RandomUniformInt(kTargetMeshCount);

  float angle = RandomUniformFloat(-M_PI, M_PI);
  float distance = RandomUniformFloat(kMinTargetDistance, kMaxTargetDistance);
  float height = RandomUniformFloat(kMinTargetHeight, kMaxTargetHeight);
  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                          std::sin(angle) * distance};

  model_target_ = GetTranslationMatrix(target_position);
}

bool CardboardBrickApp::IsPointingAtTarget() {
  // Compute vectors pointing towards the reticle and towards the target object
  // in head space.
  Matrix4x4 head_from_target = head_view_ * model_target_;

  const std::array<float, 4> unit_quaternion = {0.f, 0.f, 0.f, 1.f};
  const std::array<float, 4> point_vector = {0.f, 0.f, -1.f, 0.f};
  const std::array<float, 4> target_vector = head_from_target * unit_quaternion;

  float angle = AngleBetweenVectors(point_vector, target_vector);
  return angle < kAngleLimit;
}

}  // namespace ndk_cardboard

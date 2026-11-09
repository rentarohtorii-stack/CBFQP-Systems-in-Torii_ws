#include <map>
#include <memory>
#include <set>
#include <OgreSceneManager.h>
#include "rviz_common/display_context.hpp"
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgrePass.h>
#include <OgreRenderQueue.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "rviz_default_plugins/displays/marker_array/marker_array_display.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amir_cbf_rviz
{
// PointCloud (SPHERE_LIST) submits to the default render group regardless of
// its MovableObject queue setting. Redirect those submissions too. Share one
// listener across CBF displays, retaining the listener installed before ours.
class ForegroundQueue : public Ogre::RenderQueue::RenderableListener
{
public:
  explicit ForegroundQueue(Ogre::RenderQueue * queue)
  : queue_(queue), previous_(queue->getRenderableListener())
  {queue_->setRenderableListener(this);}

  ~ForegroundQueue() override
  {
    if (queue_->getRenderableListener() == this) {
      queue_->setRenderableListener(previous_);
    }
  }

  bool renderableQueued(Ogre::Renderable * renderable, Ogre::uint8 group,
    Ogre::ushort priority, Ogre::Technique ** technique, Ogre::RenderQueue * queue) override
  {
    bool foreground = false;
    for (const auto & owner : objects) {
      foreground = foreground || owner.second.count(renderable) != 0;
    }
    if (foreground && group != Ogre::RENDER_QUEUE_OVERLAY - 1) {
      queue->addRenderable(renderable, Ogre::RENDER_QUEUE_OVERLAY - 1, priority);
      return false;
    }
    return !previous_ || previous_->renderableQueued(renderable, group, priority, technique, queue);
  }

  std::map<const void *, std::set<Ogre::Renderable *>> objects;

private:
  Ogre::RenderQueue * queue_;
  Ogre::RenderQueue::RenderableListener * previous_;
};

std::shared_ptr<ForegroundQueue> foregroundQueue(Ogre::RenderQueue * queue)
{
  static std::map<Ogre::RenderQueue *, std::weak_ptr<ForegroundQueue>> queues;
  auto shared = queues[queue].lock();
  if (!shared) {
    shared = std::make_shared<ForegroundQueue>(queue);
    queues[queue] = shared;
  }
  return shared;
}

// RViz marker implementations own their materials. Restrict traversal to this
// display's scene subtree so ordinary OctoMap and robot displays stay unchanged.
class ForegroundMaterials : public Ogre::Renderable::Visitor
{
public:
  std::set<Ogre::Renderable *> objects;

  void visit(Ogre::Renderable * renderable, Ogre::ushort, bool, Ogre::Any *) override
  {
    objects.insert(renderable);
    const auto & material = renderable->getMaterial();
    if (!material) {
      return;
    }
    for (unsigned short t = 0; t < material->getNumTechniques(); ++t) {
      auto * technique = material->getTechnique(t);
      for (unsigned short p = 0; p < technique->getNumPasses(); ++p) {
        auto * pass = technique->getPass(p);
        pass->setDepthCheckEnabled(false);
        pass->setDepthWriteEnabled(false);
      }
    }
    // SPHERE_LIST uses PointCloud renderables as well as the parent movable.
    if (auto * movable = dynamic_cast<Ogre::MovableObject *>(renderable)) {
      movable->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    }
  }
};

void foregroundSubtree(Ogre::SceneNode * node, ForegroundMaterials & visitor)
{
  for (auto * object : node->getAttachedObjects()) {
    object->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    object->visitRenderables(&visitor);
  }
  for (auto * child : node->getChildren()) {
    foregroundSubtree(static_cast<Ogre::SceneNode *>(child), visitor);
  }
}

class ForegroundMarkerArray : public rviz_default_plugins::displays::MarkerArrayDisplay
{
public:
  ~ForegroundMarkerArray() override
  {
    if (queue_) {queue_->objects.erase(this);}
  }

  void onInitialize() override
  {
    MarkerArrayDisplay::onInitialize();
    queue_ = foregroundQueue(context_->getSceneManager()->getRenderQueue());
  }

  void onDisable() override
  {
    if (queue_) {queue_->objects.erase(this);}
    MarkerArrayDisplay::onDisable();
  }

  void reset() override
  {
    if (queue_) {queue_->objects.erase(this);}
    MarkerArrayDisplay::reset();
  }

  void update(float wall_dt, float ros_dt) override
  {
    MarkerArrayDisplay::update(wall_dt, ros_dt);
    ForegroundMaterials visitor;
    foregroundSubtree(scene_node_, visitor);
    if (queue_) {queue_->objects[this] = std::move(visitor.objects);}
  }

private:
  std::shared_ptr<ForegroundQueue> queue_;
};
}  // namespace amir_cbf_rviz

PLUGINLIB_EXPORT_CLASS(amir_cbf_rviz::ForegroundMarkerArray, rviz_common::Display)

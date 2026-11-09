#include <gtest/gtest.h>
#include <OgreRoot.h>
#include <OgreMaterialManager.h>
#include <OgreSimpleRenderable.h>
#include "../src/foreground_marker_array.cpp"

namespace
{
class TestSphere : public Ogre::SimpleRenderable
{
public:
  void assign(const Ogre::MaterialPtr & material) {mMaterial = material;}
  Ogre::Real getBoundingRadius() const override {return 1;}
  Ogre::Real getSquaredViewDepth(const Ogre::Camera *) const override {return 1;}
};

TEST(Foreground, DrawsAfterSceneWithoutDepthOcclusionAndPreservesAlpha)
{
  // No render window or graphics context needed to verify material/queue policy.
  Ogre::Root root("", "", "");
  Ogre::MaterialManager::getSingleton().initialise();
  auto material = Ogre::MaterialManager::getSingleton().create("cbf_test", "General");
  auto ordinary = Ogre::MaterialManager::getSingleton().create("octomap_test", "General");
  // Newly created materials have no techniques until explicitly configured.
  material->removeAllTechniques();
  ordinary->removeAllTechniques();
  auto * pass = material->createTechnique()->createPass();
  ordinary->createTechnique()->createPass();
  pass->setDiffuse(1.0f, 0.15f, 0.05f, 0.7f);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  TestSphere sphere;
  sphere.assign(material);
  amir_cbf_rviz::ForegroundMaterials visitor;
  visitor.visit(&sphere, 0, false, nullptr);
  EXPECT_FALSE(pass->getDepthCheckEnabled());
  EXPECT_FALSE(pass->getDepthWriteEnabled());
  EXPECT_FLOAT_EQ(pass->getDiffuse().a, 0.7f);
  EXPECT_EQ(pass->getSourceBlendFactor(), Ogre::SBF_SOURCE_ALPHA);
  EXPECT_EQ(sphere.getRenderQueueGroup(), Ogre::RENDER_QUEUE_OVERLAY - 1);
  EXPECT_GT(sphere.getRenderQueueGroup(), Ogre::RENDER_QUEUE_MAIN);
  EXPECT_TRUE(ordinary->getTechnique(0)->getPass(0)->getDepthCheckEnabled());
  // Marker updates can restore material defaults; each update must reapply policy.
  pass->setDepthCheckEnabled(true);
  visitor.visit(&sphere, 0, false, nullptr);
  EXPECT_FALSE(pass->getDepthCheckEnabled());
}
}  // namespace

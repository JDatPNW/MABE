//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

// Evaluates agents on how many '1's they can output. This is a purely fixed
// task
// that requires to reactivity to stimuli.
// Each correct '1' confers 1.0 point to score, or the decimal output determined
// by 'mode'.
#include "PhysicsWorld.h"
//#include <stdlib.h>


Eigen::Vector6d startPosition(Eigen::Vector6d::Zero());
dart::simulation::WorldPtr world(new dart::simulation::World);
dart::dynamics::SkeletonPtr sphere = dart::dynamics::Skeleton::create();
dart::gui::osg::Viewer *globalviewer;
constexpr double starting_height = 1;

bool visualization = true;

int simulationSpeed = 10;

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = 0.2;  // m

const double default_torque = 15.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

dart::dynamics::SkeletonPtr pendulum;
dart::dynamics::BodyNode* pbn;

std::shared_ptr<ParameterLink<int> > PhysicsWorld::modePL =
        Parameters::register_parameter(
                "WORLD_PHYSICS-mode", 0, "0 = bit outputs before adding, 1 = add outputs");

std::shared_ptr<ParameterLink<bool> > PhysicsWorld::visualizationPL =
        Parameters::register_parameter(
                "WORLD_PHYSICS-visualization", false, "0 = no visualization, 1 = visualization is turned on");

std::shared_ptr<ParameterLink<int> > PhysicsWorld::numberOfOutputsPL =
        Parameters::register_parameter("WORLD_PHYSICS-numberOfOutputs", 10,
                                       "number of outputs in this world");

std::shared_ptr<ParameterLink<double> > PhysicsWorld::jointLengthPL =
        Parameters::register_parameter("WORLD_PHYSICS-jointLength", 1.0,
                                       "The length of the caterpillars bodyparts");

std::shared_ptr<ParameterLink<int> > PhysicsWorld::evaluationsPerGenerationPL =
        Parameters::register_parameter("WORLD_PHYSICS-evaluationsPerGeneration", 1,
                                       "Number of times to test each Genome per "
                                       "generation (useful with non-deterministic "
                                       "brains)");

std::shared_ptr<ParameterLink<double> > PhysicsWorld::timeStepIntervalPL =
        Parameters::register_parameter("WORLD_PHYSICS-timeStepInterval", 0.05,
                                       "Number of physics steps that are being "
                                       "simulated in a given time. Lower number equals more steps.");

std::shared_ptr<ParameterLink<double> > PhysicsWorld::simulationLengthPL =
        Parameters::register_parameter("WORLD_PHYSICS-simulationLength", 2.0,
                                       "Lemgth of simulation in seconds, these are no real time.");
std::shared_ptr<ParameterLink<double> > PhysicsWorld::physicsInteractionIntervalPL =
        Parameters::register_parameter("WORLD_PHYSICS-physicsInteractionInterval", 0.25,
                                       "The interval in which the brain has influence on the simulation.");

std::shared_ptr<ParameterLink<std::string> > PhysicsWorld::groupNamePL =
        Parameters::register_parameter("WORLD_PHYSICS_NAMES-groupNameSpace",
                                       (std::string) "root::",
                                       "namespace of group to be evaluated");
std::shared_ptr<ParameterLink<std::string> > PhysicsWorld::brainNamePL =
        Parameters::register_parameter(
                "WORLD_PHYSICS_NAMES-brainNameSpace", (std::string) "root::",
                "namespace for parameters used to define brain");

PhysicsWorld::PhysicsWorld(std::shared_ptr<ParametersTable> PT_)
        : AbstractWorld(PT_) {

        // columns to be added to ave file
        popFileColumns.clear();
        popFileColumns.push_back("score");
        popFileColumns.push_back("score_VAR"); // specifies to also record the
        popFileColumns.push_back("X"); // records the X coordinate
        popFileColumns.push_back("Z"); // records the Z coordinate

        // variance (performed automatically
        // because _VAR)
        setupPhysics();
        setupVisualization();
        std::cout << std::endl << "----------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl << "Visualization Controlls:" << std::endl << std::endl;
        std::cout << "press 'q' or 'ESC' to quit visualization " << std::endl;
        std::cout << "press 'Left Arrow' to slow down visualization-speed step by step" << std::endl;
        std::cout << "press 'Right Arrow' to speed up visualization-speed step by step" << std::endl;
        std::cout << "press 'Down Arrow' to slow down visualization-speed to it's minimum" << std::endl;
        std::cout << "press 'Up Arrow' to speed up visualization-speed to it's maximum" << std::endl;
        std::cout << std::endl << "----------------------------------------------------------------------------" << std::endl;

}

class CustomEventHandler : public osgGA::GUIEventHandler
{
public:

CustomEventHandler(/*Pass in any necessary arguments*/)
{
        // Set up the customized event handler
}

virtual bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter&) override
{
        if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
                if(ea.getKey() == 'q')
                {
                        visualization = !visualization;
                        std::cout << "visualization is now " << visualization << "!" << std::endl;
                        return true;
                }
                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
                {
                        if(simulationSpeed > 10)
                                simulationSpeed -= 10;
                        std::cout << "visualization is now " << simulationSpeed << "!" << std::endl;

                        return true;
                }
                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
                {
                        simulationSpeed = 240;
                        std::cout << "visualization is now " << simulationSpeed << "!" << std::endl;

                        return true;
                }
                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
                {
                        simulationSpeed = 10;
                        std::cout << "visualization is now " << simulationSpeed << "!" << std::endl;

                        return true;
                }

                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
                {
                        if(simulationSpeed < 240)
                                simulationSpeed += 10;
                        std::cout << "visualization is now " << simulationSpeed << "!" << std::endl;
                        return true;
                }

                else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape)
                {
                        std::cout << "ESC key pressed" << std::endl;
                        return true;
                }
        }


        // The return value should be 'true' if the input has been fully handled
        // and should not be visible to any remaining event handlers. It should be
        // false if the input has not been fully handled and should be viewed by
        // any remaining event handlers.
        return false;
}

};

dart::dynamics::SkeletonPtr PhysicsWorld::createGround()
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        SkeletonPtr ground = Skeleton::create("ground");
        BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;
        std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(Eigen::Vector3d(default_ground_width, default_ground_width,default_wall_thickness));
        auto shapeNode  = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
        shapeNode->getVisualAspect()->setColor(dart::Color::Green());
        //std::cout << "Ground NumBody: " << ground->getNumBodyNodes() << std::endl;
        //std::cout << "Ground NumDof: " << ground->getNumDofs() << std::endl;
        ground->getBodyNode(0)->setFrictionCoeff(5);
        //ground->getDof(0)->setCoulombFriction(5);
        return ground;
}

dart::dynamics::SkeletonPtr PhysicsWorld::createWall()
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        SkeletonPtr wall = Skeleton::create("wall");
        BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;
        std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(Eigen::Vector3d(default_wall_thickness, default_ground_width, default_wall_height));
        auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());

        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.translation() = Eigen::Vector3d(
                (default_ground_width + default_wall_thickness)/2.0, 0.0,
                (default_wall_height  - default_wall_thickness)/2.0);
        bn->getParentJoint()->setTransformFromParentBodyNode(tf);
        bn->setRestitutionCoeff(0.2);
        return wall;
}

/*void PhysicsWorld::applyForce(dart::dynamics::BodyNode* body, double x, double y){
   Eigen::Vector3d force = (Eigen::Vector3d(x,y,0));
   Eigen::Vector3d location(0, 0.0, 0);

   body->addExtForce(force, location, false, true);
   }*/
void PhysicsWorld::applyForce(dart::dynamics::SkeletonPtr skel, std::vector<double> outputs)
{

        for(int i=0; i<outputs.size(); i++)
        {
                if(outputs.at(i) > 1.5)
                        outputs.at(i) = 1.5;
                else if(outputs.at(i) < -1.5)
                        outputs.at(i) = -1.5;
        }

        for(int i = 1; i < skel->getNumJoints(); i++)
        {
                for(std::size_t j = 0; j < skel->getJoint(i)->getNumDofs(); ++j)
                {
                        skel->getJoint(i)->getDof(j)->setRestPosition(outputs.at(i-1));
                }
        }
}

void PhysicsWorld::setupVisualization()
{
        dart::gui::osg::WorldNode * node = new dart::gui::osg::WorldNode(world);
        globalviewer = new dart::gui::osg::Viewer();
        globalviewer->addWorldNode(node);
        if(visualization && PhysicsWorld::visualizationPL->get(PT)){
          globalviewer->setUpViewInWindow(0, 0, 640, 480);
          globalviewer->realize();
          globalviewer->getCameraManipulator()->setHomePosition(::osg::Vec3( 20.57,  30.14, 10.64),::osg::Vec3( 0.00,  0.00, 0.00),::osg::Vec3(-01.24, -01.25, 01.94));
          globalviewer->setCameraManipulator(globalviewer->getCameraManipulator());
          //osg::Vec3d eye( 1000.0, 1000.0, 100.0 );
          //osg::Vec3d center( 0.0, 0.0, 0.0 );
          //osg::Vec3d up( 0.0, 0.0, 10.0 );
          //globalviewer->getCamera()->setViewMatrixAsLookAt( eye, center, up);
          globalviewer->addEventHandler(new CustomEventHandler);
          }
}

void PhysicsWorld::setGeometry(const dart::dynamics::BodyNodePtr& bn)
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        // Create a BoxShape to be used for both visualization and collision checking
        //std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(default_width, default_depth, jointLengthPL->get(PT))));
        std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(default_width, default_depth, jointLengthPL->get(PT))));
        // Create a shape node for visualization and collision checking
        auto shapeNode
                = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
        shapeNode->getVisualAspect()->setColor(dart::Color::White());

        // Set the location of the shape node
        Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
        Eigen::Vector3d center = Eigen::Vector3d(0, 0, (jointLengthPL->get(PT) / 2.0) +default_width / 2.0);
        box_tf.translation() = center;
        shapeNode->setRelativeTransform(box_tf);

        // Move the center of mass to the center of the object
        bn->setLocalCOM(center);
}

dart::dynamics::BodyNode* PhysicsWorld::makeRootBody(const dart::dynamics::SkeletonPtr& pendulum, const std::string& name)
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        BallJoint::Properties properties;
        FreeJoint::Properties freeProperties;
        properties.mName = name + "_joint";
        properties.mRestPositions = Eigen::Vector3d::Constant(default_rest_position);
        properties.mSpringStiffnesses = Eigen::Vector3d::Constant(default_stiffness);
        properties.mDampingCoefficients = Eigen::Vector3d::Constant(default_damping);

        BodyNodePtr bn = pendulum->createJointAndBodyNodePair<FreeJoint>(nullptr, freeProperties, BodyNode::AspectProperties(name)).second;
        /*
           // Make a shape for the Joint
           const double& R = default_width;
           std::shared_ptr<EllipsoidShape> ball(
           new EllipsoidShape(sqrt(2) * Eigen::Vector3d(R, R, R)));
           auto shapeNode = bn->createShapeNodeWith<VisualAspect>(ball);
           shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

           // Set the geometry of the Body
           setGeometry(bn);
         */

        // Make a shape for the Joint
        const double R = default_width / 2.0;
        const double h = default_depth;
        std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

        // Line up the cylinder with the Joint axis
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());
        shapeNode->setRelativeTransform(tf);

        // Set the geometry of the Body
        setGeometry(bn);

        return bn;
}

dart::dynamics::BodyNode* PhysicsWorld::addBody(const dart::dynamics::SkeletonPtr& pendulum, dart::dynamics::BodyNode* parent, const std::string& name)
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        // Set up the properties for the Joint
        RevoluteJoint::Properties properties;
        properties.mName = name + "_joint";
        properties.mAxis = Eigen::Vector3d::UnitY();
        properties.mT_ParentBodyToJoint.translation() =
                Eigen::Vector3d(0, 0, jointLengthPL->get(PT) + default_width);
        properties.mRestPositions[0] = default_rest_position;
        properties.mSpringStiffnesses[0] = default_stiffness;
        properties.mDampingCoefficients[0] = default_damping;

        // Create a new BodyNode, attached to its parent by a RevoluteJoint
        BodyNodePtr bn = pendulum->createJointAndBodyNodePair<RevoluteJoint>(parent, properties, BodyNode::AspectProperties(name)).second;

        // Make a shape for the Joint
        const double R = default_width / 2.0;
        const double h = default_depth;
        std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

        // Line up the cylinder with the Joint axis
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

        auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
        shapeNode->getVisualAspect()->setColor(dart::Color::Red());
        shapeNode->setRelativeTransform(tf);

        // Set the geometry of the Body
        setGeometry(bn);

        return bn;
}

void PhysicsWorld::setupPhysics()
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        constexpr double density = 1.0;
        constexpr double radius = 0.3;
        constexpr double restitution = 0.6;

        // Create the world object (shared_ptr) and add the sphere skeleton
        world = dart::simulation::World::create();
        world->addSkeleton(createGround());
        world->setTime(0);
        world->setTimeStep(timeStepIntervalPL->get(PT)); //PHYSICS SPEED

        //world->addSkeleton(pendulum);
        //pendulum->setSelfCollisionCheck(true);



        /*for(int i = 0; i < pendulum->getNumDofs(); i++)
           {
           std::cout << "Dof " << i;
           std::cout << "CoulombFriction " << i << " " << pendulum->getDof(i)->getCoulombFriction() << std::endl;
           std::cout << "Setting CoulombFriction on Dof " << i << " to " << 5 << std::endl;
           pendulum->getDof(i)->setCoulombFriction(5);
           }*/

        physicsWorld = world;
        setupCaterpillar();


}

void PhysicsWorld::setupCaterpillar()
{
        using namespace dart::dynamics;
        using namespace dart::simulation;
        //std::cout<<"TEST " << physicsWorld->getSkeleton("pendulum") << std::endl;
        //std::cout<<"TEST " << physicsWorld->getNumSkeletons() << std::endl;

        if(physicsWorld->getSkeleton("pendulum") != NULL)
                physicsWorld->removeSkeleton(pendulum);

        //if(pendulum != NULL)
        //pendulum = NULL;
        //if(pbn != NULL)
        //pbn = NULL;
        pendulum = Skeleton::create("pendulum");
        pbn = makeRootBody(pendulum, "body1");
        std::string bodyname = "body";
        for(int i=0; i<numberOfOutputsPL->get(PT)-1; i++)
        {
                bodyname = bodyname + std::to_string(i+2);
                pbn = addBody(pendulum, pbn, bodyname);
        }
        pendulum->getDof(1)->setPosition(M_PI/2);


        for(std::size_t i = 1; i < pendulum->getNumJoints(); ++i)
        {
                pendulum->getJoint(i)->setPositionLimitEnforced(true);
                for(std::size_t j = 0; j < pendulum->getJoint(i)->getNumDofs(); ++j)
                {
                        pendulum->getJoint(i)->setSpringStiffness(j, 255);
                        pendulum->getJoint(i)->setPositionLowerLimit(j,-1.5);
                        pendulum->getJoint(i)->setPositionUpperLimit(j,1.5);
                }
        }

        startPosition[5] = starting_height;
        pendulum->getDof(5)->setPosition(starting_height);

        for(int i = 0; i < pendulum->getNumBodyNodes(); i++)
        {
                //std::cout << "BodyNode " << i << " " << pendulum->getBodyNode(i) << std::endl;
                //std::cout << "FrictionCoef " << i << " " << pendulum->getBodyNode(i)->getFrictionCoeff() << std::endl;
                //std::cout << "Setting Friction on BodyNode " << i << " to " << 5 << std::endl;
                pendulum->getBodyNode(i)->setFrictionCoeff(5);
        }

        physicsWorld->addSkeleton(pendulum);

        physicsObject = pendulum;
}


void PhysicsWorld::morphBody()
{
        //physicsObject->getBodyNode(0)->getShapeNode(0)->setRelativeTransform();
        //std::cout << "physicsObject->getBodyNode(0) " << physicsObject->getBodyNode(0)->getName() << std::endl;
        //std::cout << "physicsObject->getBodyNode(0)->getShapeNode(0) " << physicsObject->getBodyNode(0)->getShapeNode(0)->getName() << std::endl;
        //dart::dynamics::SkeletonPtr testpendulum;
        //dart::dynamics::BodyNode* test pbn;

}

void PhysicsWorld::evaluate(std::map<std::string, std::shared_ptr<Group> > &groups, int analyze, int visualize, int debug) {
        int popSize = groups[groupNamePL->get(PT)]->population.size();
        for (int i = 0; i < popSize; i++) {
                evaluateSolo(groups[groupNamePL->get(PT)]->population[i], analyze,
                             visualize, debug);
        }
}

void PhysicsWorld::evaluateSolo(std::shared_ptr<Organism> org, int analyze, int visualize, int debug) {
        using namespace dart::dynamics;
        using namespace dart::simulation;
        int number = 0;
        double x, z;
        std::vector<double> brainOutputs;
        auto brain = org->brains[brainNamePL->get(PT)];
        for (int r = 0; r < evaluationsPerGenerationPL->get(PT); r++) {
                brain->resetBrain();
                physicsWorld->step(true);
                physicsWorld->reset(); //resets the counter
                //setupCaterpillar();
                //std::cout<< ".";
                //physicsObject->setPositions(startPosition);
                for(int i=0; i < physicsObject->getNumDofs(); i++)
                {
                        physicsObject->getDof(i)->setPosition(0);
                }
                if(numberOfOutputsPL->get(PT)%2 != 0)
                        physicsObject->getDof(3)->setPosition(-((numberOfOutputsPL->get(PT)/2)*((jointLengthPL->get(PT)+default_width)))); //numberOfOutputsPL->get(PT)
                else
                        physicsObject->getDof(3)->setPosition(-((numberOfOutputsPL->get(PT)/2)*((jointLengthPL->get(PT)+default_width)))+((jointLengthPL->get(PT)+default_width)*0.5));
                physicsObject->getDof(1)->setPosition(M_PI/2);
                physicsObject->getDof(5)->setPosition(starting_height);
                physicsObject->resetVelocities();
                physicsObject->resetAccelerations();

                //std::cout  << "POS 0: " <<physicsObject->getPositions()<< std::endl;
                double oldTime = 0;
                double END_TIME = simulationLengthPL->get(PT);
                while (physicsWorld->getTime() < END_TIME) {
                        if(physicsWorld->getTime() - oldTime >= physicsInteractionIntervalPL->get(PT)) {
                                brain->setInput(0, sin(physicsWorld->getTime()));
                                /*for(int i=1; i<physicsObject->getNumJoints(); i++){
                                   brain->setInput(i, physicsObject->getJoint(i)->getPosition(0));
                                   }*/
                                brain->update();
                                brainOutputs.clear();
                                for(int i=0; i<numberOfOutputsPL->get(PT); i++)
                                {
                                        brainOutputs.push_back(brain->readOutput(i));
                                }
                                applyForce(physicsObject, brainOutputs);

                                x=0;
                                z=0;
                                for(int i=0; i<physicsObject->getNumBodyNodes(); i++) {
                                        //std::cout << "Joints: " << physicsObject->getNumJoints() << std::endl;
                                        //std::cout << "Joint: " << i << " Positions: " << physicsObject->getJoint(i)->getPositions() << std::endl;
                                        x += physicsObject->getBodyNode(i)->getWorldTransform().translation()[0];
                                        //std::cout << "BodyNode_" << i << " X: " << physicsObject->getBodyNode(i)->getWorldTransform().translation()[0] << std::endl;
                                        z += physicsObject->getBodyNode(i)->getWorldTransform().translation()[2];
                                }
                                x = x/physicsObject->getNumBodyNodes();
                                z = z/physicsObject->getNumBodyNodes();
                                org->dataMap.append("X", -x);
                                org->dataMap.append("Z", z);
                                //std::cout << "X: " << x << " Z: " << z << std::endl;

                                //applyForce(physicsObject, brain->readOutput(0), brain->readOutput(1),brain->readOutput(2), brain->readOutput(3));
                                //applyForce(physicsObject, -Random::getDouble(1.5, 2), -Random::getDouble(1.5, 2), -Random::getDouble(1.5, 2), -Random::getDouble(1.5, 2));
                                oldTime = physicsWorld->getTime();
                                //std::cout << "Brain: " << brain->readOutput(0) << ", " << brain->readOutput(1) << std::endl;
                        }
                        physicsWorld->step(false);
                        if(visualization && PhysicsWorld::visualizationPL->get(PT) && number%simulationSpeed == 0)
                                globalviewer->frame();
                        number++;
                        //std::cout  << "POS: " <<physicsObject->getPositions()<< std::endl;
                }

                double mean = 0;
                for(int i=0; i<physicsObject->getNumBodyNodes(); i++) {
                        //std::cout << "Joints: " << physicsObject->getNumJoints() << std::endl;
                        //std::cout << "Joint: " << i << " Positions: " << physicsObject->getJoint(i)->getPositions() << std::endl;
                        mean += physicsObject->getBodyNode(i)->getWorldTransform().translation()[0];
                }
                mean = mean/physicsObject->getNumBodyNodes();
                double score = -mean;
                //double score = physicsObject->getPosition(5);
                if (score < 0.0)
                        score =  0.0;
                if (isnan(score))
                        score = 0.0;
                double height = 0;
                for(int i=0; i<physicsObject->getNumBodyNodes(); i++) {
                        //std::cout << "Joints: " << physicsObject->getNumJoints() << std::endl;
                        //std::cout << "Joint: " << i << " Positions: " << physicsObject->getJoint(i)->getPositions() << std::endl;
                        height += physicsObject->getBodyNode(i)->getWorldTransform().translation()[2];
                }
                height = height/physicsObject->getNumBodyNodes();
                if(height > 3)
                        score = 0.0;
                org->dataMap.append("score", score);

                //morphBody();
                //physicsWorld->removeSkeleton(pendulum);
                //pbn = addBody(pendulum, pbn, "bodyname");
                //physicsWorld->addSkeleton(pendulum);

                //std::cout << "SCORE: " << score << std::endl;
                //std::cout << "POS X: " << physicsObject->getPosition(3) << std::endl;
                //std::cout << "POS Joint 0: " << physicsObject->getJoint(0)->getPosition(3) << std::endl;
                //std::cout << "POS Y: " << physicsObject->getPosition(4) << std::endl;
                //std::cout << "POS Z: " << physicsObject->getPosition(5) << std::endl;
                //Eigen::Isometry3d test = physicsObject->getBodyNode(0)->getWorldTransform();
                //std::cout << "Transform: " << test.translation()[0] << std::endl;
        }
}

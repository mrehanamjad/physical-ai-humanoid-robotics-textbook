import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1-ros2/physical-ai-foundations',
        'module1-ros2/humanoid-robotics-overview',
        'module1-ros2/sensors-and-perception',
        'module1-ros2/ros2-architecture',
        'module1-ros2/ros2-nodes-topics-services-actions',
        'module1-ros2/python-ros2-development',
        'module1-ros2/launch-files-and-parameters',
        'module1-ros2/urdf-for-humanoid-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2-gazebo/digital-twins-and-simulation',
        'module2-gazebo/gazebo-environment-setup',
        'module2-gazebo/physics-simulation',
        'module2-gazebo/robot-description-formats-sdf-vs-urdf',
        'module2-gazebo/sensor-simulation',
        'module2-gazebo/unity-for-visualization',
        'module2-gazebo/simulation-validation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI–Robot Brain (NVIDIA Isaac)',
      items: [
        'module3-isaac/nvidia-isaac-ecosystem-overview',
        'module3-isaac/isaac-sim-and-photorealistic-simulation',
        'module3-isaac/synthetic-data-generation',
        'module3-isaac/isaac-ros-and-hardware-acceleration',
        'module3-isaac/visual-slam',
        'module3-isaac/navigation-with-nav2',
        'module3-isaac/reinforcement-learning-for-robot-control',
        'module3-isaac/sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision–Language–Action (VLA) & Humanoid Intelligence',
      items: [
        'module4-vla/humanoid-robot-kinematics-and-dynamics',
        'module4-vla/bipedal-locomotion-and-balance',
        'module4-vla/manipulation-and-grasping',
        'module4-vla/natural-human-robot-interaction',
        'module4-vla/vision-language-action-paradigms',
        'module4-vla/voice-based-control',
        'module4-vla/cognitive-planning',
        'module4-vla/capstone-project',
      ],
    },
  ],
};

export default sidebars;

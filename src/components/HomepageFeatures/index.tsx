import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

// Import icons for features
import { 
  SiRos, 
  SiUnity, 
  SiNvidia, 
  SiUbuntu,
  SiOpensourceinitiative,
  SiAmazon
} from 'react-icons/si';
import { 
  FiCpu, 
  FiCloud, 
  FiCode, 
  FiUsers,
  FiShield,
  FiTrendingUp,
  FiGlobe,
  FiActivity,
  FiWifi,
  FiHardDrive
} from 'react-icons/fi';
import { 
  RiRobot2Line, 
  RiBrainLine, 
  RiCloudLine,
  RiServerLine,
  RiCpuLine,
  RiHandHeartLine
} from 'react-icons/ri';

type FeatureItem = {
  title: string;
  icon: ReactNode;
  description: ReactNode;
  gradient: string;
  delay?: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Robotic Nervous System',
    icon: <SiRos className={styles.featureIcon} />,
    description: (
      <>
        Master ROS 2 for advanced robot control. Learn nodes, topics, services, and 
        bridge Python agents to ROS controllers using <code>rclpy</code>. Understand 
        URDF for humanoid robot description.
      </>
    ),
    gradient: 'linear-gradient(135deg, #3B82F6 0%, #1D4ED8 100%)',
    delay: '0s'
  },
  {
    title: 'Digital Twin Simulation',
    icon: <SiUnity className={styles.featureIcon} />,
    description: (
      <>
        Create physics-accurate simulations with Gazebo. Build high-fidelity 
        environments in Unity. Simulate sensors including LiDAR, Depth Cameras, 
        and IMUs for realistic testing.
      </>
    ),
    gradient: 'linear-gradient(135deg, #8B5CF6 0%, #7C3AED 100%)',
    delay: '0.1s'
  },
  {
    title: 'AI-Robot Brain',
    icon: <SiNvidia className={styles.featureIcon} />,
    description: (
      <>
        Leverage NVIDIA Isaac for photorealistic simulation and synthetic data 
        generation. Use Isaac ROS for hardware-accelerated VSLAM and Nav2 for 
        bipedal path planning.
      </>
    ),
    gradient: 'linear-gradient(135deg, #76B900 0%, #5A8C00 100%)',
    delay: '0.2s'
  },
  {
    title: 'Vision-Language-Action',
    icon: <RiBrainLine className={styles.featureIcon} />,
    description: (
      <>
        Integrate LLMs with robotics. Convert voice commands to actions using 
        Whisper. Use GPT models to translate natural language into ROS 2 
        action sequences for autonomous behavior.
      </>
    ),
    gradient: 'linear-gradient(135deg, #EF4444 0%, #DC2626 100%)',
    delay: '0.3s'
  },
  {
    title: 'Sim-to-Real Transfer',
    icon: <RiCloudLine className={styles.featureIcon} />,
    description: (
      <>
        Deploy simulation-trained models to physical robots. Master reinforcement 
        learning for robot control. Implement industry-standard safety protocols 
        and simulation-first approaches.
      </>
    ),
    gradient: 'linear-gradient(135deg, #10B981 0%, #059669 100%)',
    delay: '0.4s'
  },
  {
    title: 'Edge AI Deployment',
    icon: <RiServerLine className={styles.featureIcon} />,
    description: (
      <>
        Run real-time inference on NVIDIA Jetson Orin. Deploy to physical robots 
        with resource constraints. Understand the trade-offs between cloud and 
        edge computing for robotics.
      </>
    ),
    gradient: 'linear-gradient(135deg, #F59E0B 0%, #D97706 100%)',
    delay: '0.5s'
  },
  {
    title: 'Physical AI Hardware',
    icon: <FiCpu className={styles.featureIcon} />,
    description: (
      <>
        Work with RTX-enabled workstations, Jetson kits, and humanoid robots. 
        Set up sensor suites including RealSense cameras and IMUs for complete 
        physical AI systems.
      </>
    ),
    gradient: 'linear-gradient(135deg, #8B5CF6 0%, #7C3AED 100%)',
    delay: '0.6s'
  },
  {
    title: 'Cloud-Native Robotics',
    icon: <SiAmazon className={styles.featureIcon} />,
    description: (
      <>
        Run simulations on AWS RoboMaker and NVIDIA Omniverse Cloud. Understand 
        cost-effective cloud deployment strategies for compute-intensive 
        robotics workloads.
      </>
    ),
    gradient: 'linear-gradient(135deg, #FF9900 0%, #FF8C00 100%)',
    delay: '0.7s'
  },
  {
    title: 'Open Source Ecosystem',
    icon: <SiOpensourceinitiative className={styles.featureIcon} />,
    description: (
      <>
        Leverage the open-source robotics stack: ROS 2, Gazebo, and community 
        packages. Contribute to and learn from global robotics projects and 
        research implementations.
      </>
    ),
    gradient: 'linear-gradient(135deg, #3B82F6 0%, #1D4ED8 100%)',
    delay: '0.8s'
  }
];

function Feature({title, icon, description, gradient, delay}: FeatureItem) {
  return (
    <div className={clsx(styles.featureCard)} style={{ animationDelay: delay }}>
      <div 
        className={styles.featureIconContainer}
        style={{ background: gradient }}
      >
        {icon}
      </div>
      <div className={styles.featureContent}>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <div className={styles.featureDescription}>{description}</div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <div className={styles.sectionBadge}>Key Features</div>
          <Heading as="h2" className={styles.sectionTitle}>
            Comprehensive Physical AI Curriculum
          </Heading>
          <p className={styles.sectionDescription}>
            From simulation to deployment, master every aspect of building intelligent robots 
            that understand and interact with the physical world.
          </p>
        </div>
        
        <div className={styles.featuresGrid}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>

        {/* Additional Highlights */}
        <div className={styles.highlightsSection}>
          <div className={styles.highlightCard} style={{ 
            background: 'linear-gradient(135deg, rgba(59, 130, 246, 0.1), rgba(139, 92, 246, 0.1))',
            borderLeft: '4px solid #3B82F6'
          }}>
            <div className={styles.highlightIcon} style={{ color: '#3B82F6' }}>
              <FiUsers />
            </div>
            <div>
              <h3 className={styles.highlightTitle}>Global Community</h3>
              <p className={styles.highlightText}>
                Join thousands of learners and researchers in the Physical AI community. 
                Collaborate on open-source projects and cutting-edge research.
              </p>
            </div>
          </div>
          
          <div className={styles.highlightCard} style={{ 
            background: 'linear-gradient(135deg, rgba(16, 185, 129, 0.1), rgba(5, 150, 105, 0.1))',
            borderLeft: '4px solid #10B981'
          }}>
            <div className={styles.highlightIcon} style={{ color: '#10B981' }}>
              <FiTrendingUp />
            </div>
            <div>
              <h3 className={styles.highlightTitle}>Career Opportunities</h3>
              <p className={styles.highlightText}>
                Skills in high demand across robotics, autonomous systems, AI, and 
                industrial automation. Build projects that showcase your expertise.
              </p>
            </div>
          </div>
          
          <div className={styles.highlightCard} style={{ 
            background: 'linear-gradient(135deg, rgba(245, 158, 11, 0.1), rgba(217, 119, 6, 0.1))',
            borderLeft: '4px solid #F59E0B'
          }}>
            <div className={styles.highlightIcon} style={{ color: '#F59E0B' }}>
              <FiShield />
            </div>
            <div>
              <h3 className={styles.highlightTitle}>Safe Learning Environment</h3>
              <p className={styles.highlightText}>
                Learn with industry-standard safety protocols. Start with simulation 
                before moving to physical robots. Access to virtual labs 24/7.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
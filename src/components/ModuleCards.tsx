import React, { useState } from 'react';
import clsx from 'clsx';
import { Link } from 'react-router-dom';
import { useHistory } from '@docusaurus/router';
import styles from './ModuleCards.module.css';

// Import icons
import { 
  RiRobot2Line, 
  RiCpuLine, 
  RiBrainLine, 
  RiEyeLine,
  RiCodeLine,
  RiServerLine,
  RiGitBranchLine,
  RiCloudLine
} from 'react-icons/ri';

interface ModuleItem {
  id: string;
  title: string;
  description: string;
  link: string;
  duration: string;
  level: 'Beginner' | 'Intermediate' | 'Advanced';
  topics: string[];
  icon: React.ReactNode;
  color: string;
  progress?: number;
}

const ModuleList: ModuleItem[] = [
  {
    id: 'module-1',
    title: 'Robotic Nervous System',
    description: 'Master ROS 2 fundamentals for robot control, communication, and distributed systems',
    link: '/docs/module-1/ros-2-nodes-topics-services',
    duration: '4 weeks',
    level: 'Beginner',
    topics: ['ROS 2 Core', 'Nodes & Topics', 'Services & Actions', 'Launch Files', 'RViz'],
    icon: <RiRobot2Line />,
    color: '#4A7C59',
    progress: 100
  },
  {
    id: 'module-2',
    title: 'Digital Twin & Simulation',
    description: 'Create realistic simulations in Gazebo and Unity for safe, scalable development',
    link: '/docs/module-2/simulating-physics-in-gazebo',
    duration: '3 weeks',
    level: 'Intermediate',
    topics: ['Gazebo Simulation', 'Unity Robotics', 'URDF/SDF', 'Physics Engines', 'Visualization'],
    icon: <RiCpuLine />,
    color: '#3A6C49',
    progress: 75
  },
  {
    id: 'module-3',
    title: 'AI-Robot Brain',
    description: 'Implement AI-powered perception, navigation, and manipulation with NVIDIA Isaac',
    link: '/docs/module-3/nvidia-isaac-sim',
    duration: '5 weeks',
    level: 'Advanced',
    topics: ['NVIDIA Isaac', 'Perception AI', 'Path Planning', 'Manipulation', 'Learning'],
    icon: <RiBrainLine />,
    color: '#2C4A2C',
    progress: 50
  },
  {
    id: 'module-4',
    title: 'Vision-Language-Action',
    description: 'Integrate vision, language, and action models for intelligent robotic behaviors',
    link: '/docs/module-4/voice-to-action-whisper',
    duration: '4 weeks',
    level: 'Advanced',
    topics: ['VLA Models', 'Multi-modal AI', 'Action Planning', 'Human Interaction', 'Adaptive Control'],
    icon: <RiEyeLine />,
    color: '#4A7C59',
    progress: 25
  },
  // {
  //   id: 'module-5',
  //   title: 'Robot Programming',
  //   description: 'Advanced robot programming with Python, C++, and real-time control systems',
  //   link: '/docs/module-5/intro',
  //   duration: '3 weeks',
  //   level: 'Intermediate',
  //   topics: ['Python Robotics', 'C++ Control', 'Real-time Systems', 'Firmware', 'Debugging'],
  //   icon: <RiCodeLine />,
  //   color: '#3A6C49',
  //   progress: 10
  // },
  // {
  //   id: 'module-6',
  //   title: 'Cloud Robotics',
  //   description: 'Deploy and manage robot fleets using cloud infrastructure and edge computing',
  //   link: '/docs/module-6/intro',
  //   duration: '2 weeks',
  //   level: 'Intermediate',
  //   topics: ['Cloud Deployment', 'Fleet Management', 'Edge AI', 'OTA Updates', 'Monitoring'],
  //   icon: <RiCloudLine />,
  //   color: '#2C4A2C',
  //   progress: 0
  // }
];

interface ModuleCardProps extends ModuleItem {
  isExpanded: boolean;
  onToggle: (id: string) => void;
}

const ModuleCard = ({ 
  id,
  title, 
  description, 
  link, 
  duration, 
  level, 
  topics, 
  icon, 
  color, 
  progress = 0,
  isExpanded,
  onToggle
}: ModuleCardProps) => {
  const history = useHistory();
  const [isHovered, setIsHovered] = useState(false);

  const handleCardClick = () => {
    onToggle(id);
  };

  const handleLearnClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    history.push(link);
  };

  const getLevelColor = (level: string) => {
    switch(level) {
      case 'Beginner': return '#4A7C59';
      case 'Intermediate': return '#D87C4A';
      case 'Advanced': return '#4A7CAA';
      default: return color;
    }
  };

  const levelColor = getLevelColor(level);

  return (
    <div 
      className={clsx(
        styles.moduleCard, 
        isExpanded && styles.expanded,
        isHovered && styles.hovered
      )}
      onClick={handleCardClick}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      style={{ borderLeftColor: color }}
    >
      {/* Card Header */}
      <div className={styles.cardHeader}>
        <div className={styles.iconContainer} style={{ backgroundColor: `${color}20`, color }}>
          {icon}
        </div>
        <div className={styles.headerContent}>
          <div className={styles.titleRow}>
            <h3 className={styles.title}>{title}</h3>
            <div className={styles.badge} style={{ backgroundColor: levelColor }}>
              {level}
            </div>
          </div>
          <div className={styles.metaInfo}>
            <span className={styles.duration}>⏱️ {duration}</span>
            <span className={styles.topicsCount}>📚 {topics.length} topics</span>
          </div>
        </div>
        <button 
          className={clsx(styles.expandButton, isExpanded && styles.expanded)}
          onClick={(e) => {
            e.stopPropagation();
            handleCardClick();
          }}
        >
          {isExpanded ? '−' : '+'}
        </button>
      </div>

      {/* Card Body */}
      <div className={styles.cardBody}>
        <p className={styles.description}>{description}</p>
        
        {/* Progress Bar */}
        <div className={styles.progressContainer}>
          <div className={styles.progressLabels}>
            <span>Progress</span>
            <span className={styles.progressPercentage}>{progress}%</span>
          </div>
          <div className={styles.progressBar}>
            <div 
              className={styles.progressFill} 
              style={{ 
                width: `${progress}%`,
                backgroundColor: color
              }}
            />
          </div>
        </div>

        {/* Topics List (Only shown when expanded) */}
        {isExpanded && (
          <div className={styles.expandedContent}>
            <h4 className={styles.topicsTitle}>Topics Covered:</h4>
            <div className={styles.topicsGrid}>
              {topics.map((topic, index) => (
                <div key={index} className={styles.topicItem}>
                  <span className={styles.topicIcon}>✓</span>
                  <span className={styles.topicText}>{topic}</span>
                </div>
              ))}
            </div>
            
            {/* Prerequisites */}
            <div className={styles.prerequisites}>
              <h5 className={styles.prerequisitesTitle}>Prerequisites:</h5>
              <ul className={styles.prerequisitesList}>
                <li>Basic Python programming</li>
                <li>Linux command line</li>
                {level === 'Advanced' && <li>Previous modules or equivalent experience</li>}
              </ul>
            </div>
          </div>
        )}

        {/* Action Buttons */}
        <div className={styles.cardActions}>
          <button 
            className={styles.learnButton}
            onClick={handleLearnClick}
            style={{ backgroundColor: color }}
          >
            Start Module
          </button>
          <button className={styles.previewButton}>
            Preview Lesson
          </button>
        </div>
      </div>

      {/* Card Footer */}
      <div className={styles.cardFooter}>
        <div className={styles.learningStats}>
          <span className={styles.stat}>
            <RiGitBranchLine className={styles.statIcon} />
            <span className={styles.statValue}>4</span>
            <span className={styles.statLabel}>Projects</span>
          </span>
          <span className={styles.stat}>
            <RiServerLine className={styles.statIcon} />
            <span className={styles.statValue}>12</span>
            <span className={styles.statLabel}>Labs</span>
          </span>
          <span className={styles.stat}>
            <RiCpuLine className={styles.statIcon} />
            <span className={styles.statValue}>{topics.length}</span>
            <span className={styles.statLabel}>Topics</span>
          </span>
        </div>
      </div>
    </div>
  );
};

interface ModuleFilter {
  level: string[];
  duration: string[];
}

export default function ModuleCards(): JSX.Element {
  const [expandedCard, setExpandedCard] = useState<string | null>(null);
  const [filter, setFilter] = useState<ModuleFilter>({
    level: [],
    duration: []
  });
  const [sortBy, setSortBy] = useState<'default' | 'level' | 'duration' | 'progress'>('default');

  const handleToggleCard = (id: string) => {
    setExpandedCard(expandedCard === id ? null : id);
  };

  const toggleLevelFilter = (level: string) => {
    setFilter(prev => ({
      ...prev,
      level: prev.level.includes(level) 
        ? prev.level.filter(l => l !== level)
        : [...prev.level, level]
    }));
  };

  const toggleDurationFilter = (duration: string) => {
    setFilter(prev => ({
      ...prev,
      duration: prev.duration.includes(duration) 
        ? prev.duration.filter(d => d !== duration)
        : [...prev.duration, duration]
    }));
  };

  // Filter modules based on active filters
  const filteredModules = ModuleList.filter(module => {
    if (filter.level.length > 0 && !filter.level.includes(module.level)) {
      return false;
    }
    if (filter.duration.length > 0 && !filter.duration.some(d => module.duration.includes(d))) {
      return false;
    }
    return true;
  });

  // Sort modules
  const sortedModules = [...filteredModules].sort((a, b) => {
    switch(sortBy) {
      case 'level':
        const levelOrder = { 'Beginner': 0, 'Intermediate': 1, 'Advanced': 2 };
        return levelOrder[a.level] - levelOrder[b.level];
      case 'duration':
        const aDuration = parseInt(a.duration);
        const bDuration = parseInt(b.duration);
        return aDuration - bDuration;
      case 'progress':
        return (b.progress || 0) - (a.progress || 0);
      default:
        return 0;
    }
  });

  const levels = ['Beginner', 'Intermediate', 'Advanced'];
  const durations = ['2 weeks', '3 weeks', '4 weeks', '5 weeks'];

  return (
    <section className={styles.moduleCards}>
      <div className="container">
        {/* Header with Stats */}
        <div className={styles.header}>
          <div className={styles.headerContent}>
            <h2 className={styles.sectionTitle}>Course Modules</h2>
            <p className={styles.sectionDescription}>
              A comprehensive journey from basic robotics to advanced Physical AI systems. 
              Complete all modules to become a certified Physical AI Engineer.
            </p>
          </div>
          <div className={styles.statsOverview}>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>{ModuleList.length}</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>
                {Math.round(ModuleList.reduce((acc, m) => acc + (m.progress || 0), 0) / ModuleList.length)}%
              </div>
              <div className={styles.statLabel}>Avg. Progress</div>
            </div>
            <div className={styles.statCard}>
              <div className={styles.statNumber}>
                {ModuleList.reduce((acc, m) => acc + m.topics.length, 0)}
              </div>
              <div className={styles.statLabel}>Topics</div>
            </div>
          </div>
        </div>

        {/* Filter and Sort Controls */}
        <div className={styles.controls}>
          <div className={styles.filterGroup}>
            <span className={styles.filterLabel}>Filter by:</span>
            <div className={styles.filterButtons}>
              {levels.map(level => (
                <button
                  key={level}
                  className={clsx(
                    styles.filterButton,
                    filter.level.includes(level) && styles.active
                  )}
                  onClick={() => toggleLevelFilter(level)}
                  style={{
                    backgroundColor: filter.level.includes(level) ? getLevelColor(level) : 'transparent',
                    borderColor: getLevelColor(level)
                  }}
                >
                  {level}
                </button>
              ))}
            </div>
            <div className={styles.filterButtons}>
              {durations.map(duration => (
                <button
                  key={duration}
                  className={clsx(
                    styles.filterButton,
                    filter.duration.includes(duration) && styles.active
                  )}
                  onClick={() => toggleDurationFilter(duration)}
                >
                  {duration}
                </button>
              ))}
            </div>
          </div>
          
          <div className={styles.sortGroup}>
            <span className={styles.sortLabel}>Sort by:</span>
            <select 
              className={styles.sortSelect}
              value={sortBy}
              onChange={(e) => setSortBy(e.target.value as any)}
            >
              <option value="default">Recommended</option>
              <option value="level">Difficulty</option>
              <option value="duration">Duration</option>
              <option value="progress">Progress</option>
            </select>
          </div>
        </div>

        {/* Reset Filters Button */}
        {(filter.level.length > 0 || filter.duration.length > 0) && (
          <div className={styles.resetContainer}>
            <button 
              className={styles.resetButton}
              onClick={() => setFilter({ level: [], duration: [] })}
            >
              Clear all filters
            </button>
            <span className={styles.filterCount}>
              {filteredModules.length} of {ModuleList.length} modules shown
            </span>
          </div>
        )}

        {/* Modules Grid */}
        <div className={styles.modulesGrid}>
          {sortedModules.map(module => (
            <ModuleCard
              key={module.id}
              {...module}
              isExpanded={expandedCard === module.id}
              onToggle={handleToggleCard}
            />
          ))}
        </div>

        {/* Completion Progress */}
        <div className={styles.completionSection}>
          <div className={styles.completionHeader}>
            <h3 className={styles.completionTitle}>Course Completion</h3>
            <div className={styles.completionPercentage}>
              {Math.round(ModuleList.reduce((acc, m) => acc + (m.progress || 0), 0) / ModuleList.length)}%
            </div>
          </div>
          <div className={styles.completionBar}>
            {ModuleList.map((module, index) => (
              <div 
                key={module.id}
                className={styles.moduleSegment}
                style={{
                  width: `${100 / ModuleList.length}%`,
                  backgroundColor: module.progress === 100 ? module.color : `${module.color}40`
                }}
                title={`${module.title}: ${module.progress}%`}
              />
            ))}
          </div>
          <div className={styles.completionLabels}>
            {ModuleList.map(module => (
              <span key={module.id} className={styles.completionLabel}>
                {module.title.split(':')[0]}
              </span>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

// Helper function for level colors
function getLevelColor(level: string): string {
  switch(level) {
    case 'Beginner': return '#4A7C59';
    case 'Intermediate': return '#D87C4A';
    case 'Advanced': return '#4A7CAA';
    default: return '#4A7C59';
  }
}
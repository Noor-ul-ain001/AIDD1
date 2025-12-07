import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './HardwareTable.module.css';
import { Link } from 'react-router-dom';

// Import icons
import { 
  FiCpu, 
  FiMonitor, 
  FiServer, 
  FiCamera, 
  FiRadio,
  FiCpu as FiGpu,
  FiHardDrive,
  FiDollarSign,
  FiShoppingCart,
  FiFilter,
  FiDownload,
  FiZoomIn,
  FiRotateCw,
  FiChevronRight,
  FiShoppingBag,
  FiTool,
  FiWifi,
  FiZap
} from 'react-icons/fi';
import { RiRobot2Line, RiShoppingCartLine, RiExchangeDollarLine } from 'react-icons/ri';

// Hardware categories
type HardwareCategory = 'all' | 'workstation' | 'edge' | 'robot' | 'sensors' | 'accessories';
type ViewMode = 'grid' | 'list' | 'comparison';

interface HardwareItem {
  id: string;
  name: string;
  category: HardwareCategory;
  description: string;
  specs: {
    cpu: string;
    gpu: string;
    ram: string;
    storage: string;
    power: string;
    connectivity: string;
    os: string;
  };
  models: Array<{
    name: string;
    manufacturer: string;
    modelUrl: string;
    imageUrl: string;
    is3D: boolean;
  }>;
  cost: {
    min: number;
    max: number;
    currency: string;
    bestFor: string;
  };
  features: string[];
  compatibility: string[];
  purchaseLinks: Array<{
    name: string;
    url: string;
    price: number;
  }>;
  pros: string[];
  cons: string[];
  popularity: number; // 1-100
  energyEfficiency: 'A' | 'B' | 'C' | 'D' | 'E';
  upgradePath: string[];
}

const HardwareList: HardwareItem[] = [
  {
    id: 'workstation-1',
    name: 'AI Workstation Pro',
    category: 'workstation',
    description: 'High-performance desktop optimized for AI model training, simulation, and robotics development',
    specs: {
      cpu: 'Intel Core i9-14900K / AMD Ryzen 9 7950X',
      gpu: 'NVIDIA RTX 4090 (24GB) or dual RTX 4090',
      ram: '64GB - 128GB DDR5',
      storage: '2TB NVMe SSD + 4TB HDD',
      power: '1000W 80+ Platinum',
      connectivity: 'WiFi 6E, 10GbE, Thunderbolt 4',
      os: 'Ubuntu 22.04 LTS / Windows 11 Pro'
    },
    models: [
      { name: 'NVIDIA RTX 4090', manufacturer: 'NVIDIA', modelUrl: '/models/rtx4090.glb', imageUrl: 'https://images.unsplash.com/photo-1593640408182-31c70c8268f5', is3D: true },
      { name: 'AMD Ryzen 9', manufacturer: 'AMD', modelUrl: '/models/ryzen9.glb', imageUrl: 'https://images.unsplash.com/photo-1587202372616-b43abea06c2a', is3D: true }
    ],
    cost: {
      min: 3000,
      max: 8000,
      currency: 'USD',
      bestFor: 'Heavy AI training & simulation'
    },
    features: ['Liquid cooling', 'Overclocking support', 'Dual GPU support', 'Professional workstation certification'],
    compatibility: ['ROS 2', 'NVIDIA Isaac', 'PyTorch', 'TensorFlow', 'Gazebo', 'Unity'],
    purchaseLinks: [
      { name: 'Amazon', url: '#', price: 3500 },
      { name: 'Newegg', url: '#', price: 3400 },
      { name: 'Micro Center', url: '#', price: 3200 }
    ],
    pros: ['Excellent for training', 'Future-proof', 'Great for simulation'],
    cons: ['Expensive', 'High power consumption', 'Large footprint'],
    popularity: 85,
    energyEfficiency: 'C',
    upgradePath: ['GPU', 'RAM', 'Storage']
  },
  {
    id: 'edge-1',
    name: 'NVIDIA Jetson AGX Orin',
    category: 'edge',
    description: 'Compact AI computing platform for embedded and edge devices with GPU acceleration',
    specs: {
      cpu: '12-core ARM Cortex-A78AE',
      gpu: '2048-core NVIDIA Ampere GPU',
      ram: '32GB LPDDR5',
      storage: '64GB eMMC + NVMe expansion',
      power: '15W - 60W',
      connectivity: 'WiFi 6, 5G, CAN, Ethernet',
      os: 'JetPack 5.0 (Ubuntu 20.04)'
    },
    models: [
      { name: 'Jetson AGX Orin', manufacturer: 'NVIDIA', modelUrl: '/models/jetson-orin.glb', imageUrl: 'https://images.unsplash.com/photo-1518709268805-4e9042af2176', is3D: true },
      { name: 'Developer Kit', manufacturer: 'NVIDIA', modelUrl: '/models/jetson-devkit.glb', imageUrl: 'https://images.unsplash.com/photo-1591799264318-7e6ef8ddb7ea', is3D: true }
    ],
    cost: {
      min: 1500,
      max: 3000,
      currency: 'USD',
      bestFor: 'Edge AI deployment'
    },
    features: ['AI performance: 275 TOPS', 'Hardware accelerators', 'Multiple sensor support', 'Robotics SDK'],
    compatibility: ['ROS 2', 'NVIDIA Isaac', 'TensorRT', 'DeepStream', 'VPI'],
    purchaseLinks: [
      { name: 'NVIDIA Store', url: '#', price: 1999 },
      { name: 'Arrow Electronics', url: '#', price: 1950 },
      { name: 'Mouser', url: '#', price: 2100 }
    ],
    pros: ['High performance per watt', 'Compact form factor', 'Robotics optimized'],
    cons: ['Limited upgrade options', 'Learning curve', 'Requires carrier board'],
    popularity: 92,
    energyEfficiency: 'A',
    upgradePath: ['Carrier board', 'Storage', 'Cooling']
  },
  {
    id: 'robot-1',
    name: 'Unitree Go2 Edu',
    category: 'robot',
    description: 'Advanced quadruped robot platform for research and education with full ROS 2 support',
    specs: {
      cpu: 'Intel NUC / Jetson Xavier NX',
      gpu: 'Integrated / Discrete GPU option',
      ram: '8GB - 32GB',
      storage: '512GB SSD',
      power: '48V Li-ion battery (120 min)',
      connectivity: 'WiFi, Ethernet, Bluetooth',
      os: 'Ubuntu 20.04 + ROS 2'
    },
    models: [
      { name: 'Unitree Go2', manufacturer: 'Unitree', modelUrl: '/models/unitree-go2.glb', imageUrl: 'https://images.unsplash.com/photo-1677442136019-21780ecad995', is3D: true },
      { name: 'Go2 Edu Edition', manufacturer: 'Unitree', modelUrl: '/models/go2-edu.glb', imageUrl: 'https://images.unsplash.com/photo-1620712943543-bcc4688e7485', is3D: true }
    ],
    cost: {
      min: 10000,
      max: 30000,
      currency: 'USD',
      bestFor: 'Physical AI research'
    },
    features: ['360° perception', 'Dynamic locomotion', 'Payload: 5kg', 'Multiple gaits', 'Fall recovery'],
    compatibility: ['ROS 2', 'Python/C++ API', 'Gazebo simulation', 'MATLAB/Simulink'],
    purchaseLinks: [
      { name: 'Unitree Store', url: '#', price: 15000 },
      { name: 'Academic Discount', url: '#', price: 12000 }
    ],
    pros: ['Excellent mobility', 'Research ready', 'Good documentation'],
    cons: ['Very expensive', 'Maintenance costs', 'Requires space'],
    popularity: 78,
    energyEfficiency: 'B',
    upgradePath: ['Sensors', 'Compute unit', 'Battery']
  },
  {
    id: 'sensor-1',
    name: 'Intel RealSense D455',
    category: 'sensors',
    description: 'Stereo depth camera with wide field of view, ideal for robotics and 3D scanning',
    specs: {
      cpu: 'Integrated VPU',
      gpu: 'N/A',
      ram: 'N/A',
      storage: 'N/A',
      power: 'USB powered (3.5W)',
      connectivity: 'USB 3.2 Type-C',
      os: 'Windows/Linux/ROS'
    },
    models: [
      { name: 'RealSense D455', manufacturer: 'Intel', modelUrl: '/models/realsense.glb', imageUrl: 'https://images.unsplash.com/photo-1581094794329-c8112a89af12', is3D: true }
    ],
    cost: {
      min: 400,
      max: 600,
      currency: 'USD',
      bestFor: 'Depth perception & SLAM'
    },
    features: ['RGB-D camera', 'Global shutter', 'IMU', '6DoF tracking', 'ROS 2 drivers'],
    compatibility: ['ROS 2', 'OpenCV', 'PyTorch3D', 'Open3D'],
    purchaseLinks: [
      { name: 'Intel Store', url: '#', price: 499 },
      { name: 'Amazon', url: '#', price: 520 },
      { name: 'B&H Photo', url: '#', price: 510 }
    ],
    pros: ['Accurate depth', 'Good SDK', 'Wide FOV'],
    cons: ['Limited range', 'Light sensitive', 'USB bandwidth'],
    popularity: 88,
    energyEfficiency: 'A',
    upgradePath: ['Lens options', 'Mounting hardware']
  },
  {
    id: 'edge-2',
    name: 'NVIDIA Jetson Orin Nano',
    category: 'edge',
    description: 'Entry-level AI computing module perfect for smaller robots and IoT devices',
    specs: {
      cpu: '6-core ARM Cortex-A78AE',
      gpu: '512-core NVIDIA Ampere GPU',
      ram: '4GB - 8GB LPDDR5',
      storage: '32GB eMMC',
      power: '5W - 15W',
      connectivity: 'WiFi, Bluetooth, CAN, I2C, SPI',
      os: 'JetPack 5.0'
    },
    models: [
      { name: 'Jetson Orin Nano', manufacturer: 'NVIDIA', modelUrl: '/models/orin-nano.glb', imageUrl: 'https://images.unsplash.com/photo-1518709268805-4e9042af2176', is3D: true }
    ],
    cost: {
      min: 499,
      max: 799,
      currency: 'USD',
      bestFor: 'Small robots & drones'
    },
    features: ['40 TOPS AI performance', 'Small form factor', 'Low power consumption', 'Robot ready'],
    compatibility: ['ROS 2', 'NVIDIA Isaac', 'TensorRT', 'OpenCV'],
    purchaseLinks: [
      { name: 'NVIDIA Store', url: '#', price: 599 },
      { name: 'Seeed Studio', url: '#', price: 620 },
      { name: 'SparkFun', url: '#', price: 610 }
    ],
    pros: ['Affordable', 'Energy efficient', 'Good performance'],
    cons: ['Limited memory', 'Fewer ports', 'Basic features'],
    popularity: 90,
    energyEfficiency: 'A+',
    upgradePath: ['Carrier boards', 'Cooling', 'Storage']
  },
  {
    id: 'accessory-1',
    name: 'Robot Development Kit',
    category: 'accessories',
    description: 'Complete kit with sensors, motors, and controllers for custom robot building',
    specs: {
      cpu: 'Various MCUs',
      gpu: 'N/A',
      ram: 'Varies',
      storage: 'MicroSD',
      power: 'Multiple voltage rails',
      connectivity: 'WiFi, Bluetooth, Serial',
      os: 'Arduino/ESP32/ROS'
    },
    models: [
      { name: 'Robot Kit', manufacturer: 'Various', modelUrl: '/models/robot-kit.glb', imageUrl: 'https://images.unsplash.com/photo-1581094794329-c8112a89af12', is3D: true }
    ],
    cost: {
      min: 500,
      max: 2000,
      currency: 'USD',
      bestFor: 'Custom robot projects'
    },
    features: ['Multiple sensors', 'Servo motors', 'Motor controllers', 'Chassis', 'Battery pack'],
    compatibility: ['ROS 2', 'Arduino', 'ESP32', 'Python'],
    purchaseLinks: [
      { name: 'Amazon', url: '#', price: 750 },
      { name: 'RobotShop', url: '#', price: 800 },
      { name: 'Adafruit', url: '#', price: 900 }
    ],
    pros: ['Versatile', 'Educational', 'All-in-one'],
    cons: ['Assembly required', 'Varying quality', 'Limited support'],
    popularity: 75,
    energyEfficiency: 'B',
    upgradePath: ['Add sensors', 'Better motors', 'Advanced compute']
  }
];

const categories = [
  { id: 'all', name: 'All Hardware', icon: <FiServer />, color: '#4A7C59' },
  { id: 'workstation', name: 'Workstations', icon: <FiMonitor />, color: '#3A6C49' },
  { id: 'edge', name: 'Edge Computing', icon: <FiCpu />, color: '#4A7C59' },
  { id: 'robot', name: 'Robots', icon: <RiRobot2Line />, color: '#2C4A2C' },
  { id: 'sensors', name: 'Sensors', icon: <FiCamera />, color: '#3A6C49' },
  { id: 'accessories', name: 'Accessories', icon: <FiTool />, color: '#4A7C59' }
];

// Price ranges
const priceRanges = [
  { id: 'all', label: 'Any Price' },
  { id: 'budget', label: 'Under $1,000', max: 1000 },
  { id: 'mid', label: '$1,000 - $5,000', min: 1000, max: 5000 },
  { id: 'high', label: '$5,000 - $15,000', min: 5000, max: 15000 },
  { id: 'premium', label: 'Over $15,000', min: 15000 }
];

interface FilterState {
  category: HardwareCategory;
  priceRange: string;
  popularity: number;
  energyEfficiency: string[];
}

const HardwareCard = ({ hardware, isComparing, onCompareToggle }: {
  hardware: HardwareItem;
  isComparing: boolean;
  onCompareToggle: (id: string) => void;
}) => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [selectedModel, setSelectedModel] = useState(0);
  const [isModelViewerOpen, setIsModelViewerOpen] = useState(false);

  const handlePurchaseClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    // In a real app, this would open purchase modal
    console.log('Purchase clicked for', hardware.name);
  };

  const formatPrice = (min: number, max: number) => {
    if (min === max) return `$${min}`;
    return `$${min} - $${max}`;
  };

  const getEfficiencyColor = (rating: string) => {
    switch(rating) {
      case 'A': return '#4A7C59';
      case 'B': return '#8BC34A';
      case 'C': return '#FFC107';
      case 'D': return '#FF9800';
      case 'E': return '#F44336';
      default: return '#9E9E9E';
    }
  };

  const efficiencyColor = getEfficiencyColor(hardware.energyEfficiency);

  return (
    <div 
      className={clsx(
        styles.hardwareCard, 
        isExpanded && styles.expanded,
        isComparing && styles.comparing
      )}
      onClick={() => setIsExpanded(!isExpanded)}
    >
      {/* Card Header */}
      <div className={styles.cardHeader}>
        <div className={styles.categoryBadge} style={{ backgroundColor: getCategoryColor(hardware.category) }}>
          {getCategoryIcon(hardware.category)}
          <span>{getCategoryName(hardware.category)}</span>
        </div>
        <button 
          className={clsx(styles.compareButton, isComparing && styles.active)}
          onClick={(e) => {
            e.stopPropagation();
            onCompareToggle(hardware.id);
          }}
          title={isComparing ? 'Remove from comparison' : 'Add to comparison'}
        >
        </button>
      </div>

      {/* Card Body */}
      <div className={styles.cardBody}>
        <div className={styles.hardwareImage}>
          <div className={styles.imagePlaceholder}>
            <div className={styles.imageOverlay}>
              <FiZoomIn className={styles.zoomIcon} />
              <span>3D Model</span>
            </div>
          </div>
          <div className={styles.modelSelector}>
            {hardware.models.map((model, index) => (
              <button
                key={index}
                className={clsx(styles.modelButton, selectedModel === index && styles.active)}
                onClick={(e) => {
                  e.stopPropagation();
                  setSelectedModel(index);
                }}
              >
                {model.name}
              </button>
            ))}
          </div>
        </div>

        <div className={styles.hardwareInfo}>
          <h3 className={styles.hardwareName}>{hardware.name}</h3>
          <p className={styles.hardwareDescription}>{hardware.description}</p>
          
          <div className={styles.ratingBars}>
            <div className={styles.ratingItem}>
              <span className={styles.ratingLabel}>Popularity</span>
              <div className={styles.ratingBar}>
                <div 
                  className={styles.ratingFill} 
                  style={{ width: `${hardware.popularity}%` }}
                />
              </div>
              <span className={styles.ratingValue}>{hardware.popularity}%</span>
            </div>
            <div className={styles.ratingItem}>
              <span className={styles.ratingLabel}>Energy Efficiency</span>
              <div 
                className={styles.efficiencyBadge}
                style={{ backgroundColor: efficiencyColor }}
              >
                {hardware.energyEfficiency}
              </div>
            </div>
          </div>

          <div className={styles.priceSection}>
            <div className={styles.priceTag}>
              <FiDollarSign className={styles.priceIcon} />
              <span className={styles.price}>{formatPrice(hardware.cost.min, hardware.cost.max)}</span>
            </div>
            <span className={styles.bestFor}>Best for: {hardware.cost.bestFor}</span>
          </div>
        </div>
      </div>

      {/* Quick Specs */}
      <div className={styles.quickSpecs}>
        <div className={styles.specItem}>
          <FiCpu className={styles.specIcon} />
          <span className={styles.specValue}>{hardware.specs.cpu.split('/')[0]}</span>
        </div>
        <div className={styles.specItem}>
          <FiGpu className={styles.specIcon} />
          <span className={styles.specValue}>{hardware.specs.gpu.split('(')[0]}</span>
        </div>
        <div className={styles.specItem}>
          
          <span className={styles.specValue}>{hardware.specs.ram}</span>
        </div>
      </div>

      {/* Expanded Details */}
      {isExpanded && (
        <div className={styles.expandedDetails}>
          <div className={styles.detailsGrid}>
            <div className={styles.detailsColumn}>
              <h4 className={styles.detailsTitle}>Specifications</h4>
              <div className={styles.specsList}>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>CPU</span>
                  <span className={styles.specValue}>{hardware.specs.cpu}</span>
                </div>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>GPU</span>
                  <span className={styles.specValue}>{hardware.specs.gpu}</span>
                </div>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>RAM</span>
                  <span className={styles.specValue}>{hardware.specs.ram}</span>
                </div>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>Storage</span>
                  <span className={styles.specValue}>{hardware.specs.storage}</span>
                </div>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>Power</span>
                  <span className={styles.specValue}>{hardware.specs.power}</span>
                </div>
                <div className={styles.specRow}>
                  <span className={styles.specLabel}>OS</span>
                  <span className={styles.specValue}>{hardware.specs.os}</span>
                </div>
              </div>
            </div>

            <div className={styles.detailsColumn}>
              <h4 className={styles.detailsTitle}>Features</h4>
              <ul className={styles.featuresList}>
                {hardware.features.map((feature, index) => (
                  <li key={index} className={styles.featureItem}>
                    <FiChevronRight className={styles.featureIcon} />
                    {feature}
                  </li>
                ))}
              </ul>
            </div>

            <div className={styles.detailsColumn}>
              <h4 className={styles.detailsTitle}>Compatibility</h4>
              <div className={styles.compatibilityTags}>
                {hardware.compatibility.map((item, index) => (
                  <span key={index} className={styles.compatibilityTag}>
                    {item}
                  </span>
                ))}
              </div>

              <h4 className={styles.detailsTitle}>Pros & Cons</h4>
              <div className={styles.prosCons}>
                <div className={styles.prosList}>
                  <h5>Pros</h5>
                  {hardware.pros.map((pro, index) => (
                    <div key={index} className={styles.proItem}>✓ {pro}</div>
                  ))}
                </div>
                <div className={styles.consList}>
                  <h5>Cons</h5>
                  {hardware.cons.map((con, index) => (
                    <div key={index} className={styles.conItem}>✗ {con}</div>
                  ))}
                </div>
              </div>
            </div>
          </div>

          {/* Purchase Options */}
          <div className={styles.purchaseSection}>
            <h4 className={styles.purchaseTitle}>Where to Buy</h4>
            <div className={styles.purchaseOptions}>
              {hardware.purchaseLinks.map((link, index) => (
                <a
                  key={index}
                  href={link.url}
                  className={styles.purchaseOption}
                  target="_blank"
                  rel="noopener noreferrer"
                  onClick={handlePurchaseClick}
                >
                  <span className={styles.vendorName}>{link.name}</span>
                  <span className={styles.vendorPrice}>${link.price}</span>
                  <FiShoppingCart className={styles.cartIcon} />
                </a>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Card Footer */}
      <div className={styles.cardFooter}>
        <button 
          className={styles.view3DButton}
          onClick={(e) => {
            e.stopPropagation();
            setIsModelViewerOpen(true);
          }}
        >
          <FiZoomIn className={styles.buttonIcon} />
          View 3D Model
        </button>
        <button 
          className={styles.purchaseButton}
          onClick={handlePurchaseClick}
        >
          <FiShoppingBag className={styles.buttonIcon} />
          Buy Now
        </button>
      </div>
    </div>
  );
};

// Helper functions
const getCategoryIcon = (category: HardwareCategory) => {
  switch(category) {
    case 'workstation': return <FiMonitor />;
    case 'edge': return <FiCpu />;
    case 'robot': return <RiRobot2Line />;
    case 'sensors': return <FiCamera />;
    case 'accessories': return <FiTool />;
    default: return <FiServer />;
  }
};

const getCategoryName = (category: HardwareCategory) => {
  switch(category) {
    case 'workstation': return 'Workstation';
    case 'edge': return 'Edge Computing';
    case 'robot': return 'Robot';
    case 'sensors': return 'Sensors';
    case 'accessories': return 'Accessories';
    default: return 'Hardware';
  }
};

const getCategoryColor = (category: HardwareCategory) => {
  switch(category) {
    case 'workstation': return '#4A7C59';
    case 'edge': return '#3A6C49';
    case 'robot': return '#2C4A2C';
    case 'sensors': return '#3A6C49';
    case 'accessories': return '#4A7C59';
    default: return '#4A7C59';
  }
};

const HardwareTable = () => {
  const [filter, setFilter] = useState<FilterState>({
    category: 'all',
    priceRange: 'all',
    popularity: 0,
    energyEfficiency: []
  });
  const [viewMode, setViewMode] = useState<ViewMode>('grid');
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedHardware, setSelectedHardware] = useState<string[]>([]);
  const [budget, setBudget] = useState<number>(5000);
  const [isFiltersOpen, setIsFiltersOpen] = useState(true);

  // Filter hardware based on state
  const filteredHardware = HardwareList.filter(hardware => {
    // Category filter
    if (filter.category !== 'all' && hardware.category !== filter.category) {
      return false;
    }

    // Price range filter
    if (filter.priceRange !== 'all') {
      const priceRange = priceRanges.find(r => r.id === filter.priceRange);
      if (priceRange) {
        const maxPrice = hardware.cost.max;
        if (priceRange.min && maxPrice < priceRange.min) return false;
        if (priceRange.max && hardware.cost.min > priceRange.max) return false;
      }
    }

    // Popularity filter
    if (filter.popularity > 0 && hardware.popularity < filter.popularity) {
      return false;
    }

    // Energy efficiency filter
    if (filter.energyEfficiency.length > 0 && 
        !filter.energyEfficiency.includes(hardware.energyEfficiency)) {
      return false;
    }

    // Search query filter
    if (searchQuery && !hardware.name.toLowerCase().includes(searchQuery.toLowerCase()) &&
        !hardware.description.toLowerCase().includes(searchQuery.toLowerCase())) {
      return false;
    }

    // Budget filter
    if (hardware.cost.min > budget) {
      return false;
    }

    return true;
  });

  const handleCompareToggle = (id: string) => {
    setSelectedHardware(prev => 
      prev.includes(id) 
        ? prev.filter(item => item !== id)
        : [...prev, id]
    );
  };

  const calculateTotalCost = () => {
    return selectedHardware.reduce((total, id) => {
      const item = HardwareList.find(h => h.id === id);
      return total + (item ? item.cost.min : 0);
    }, 0);
  };

  const clearFilters = () => {
    setFilter({
      category: 'all',
      priceRange: 'all',
      popularity: 0,
      energyEfficiency: []
    });
    setSearchQuery('');
    setBudget(5000);
  };

  const efficiencyOptions = ['A', 'B', 'C', 'D', 'E'];

  return (
    <section className={styles.hardwareTable}>
      <div className="container">
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerContent}>
            <h2 className={styles.title}>Hardware Visualization</h2>
            <p className={styles.subtitle}>
              Explore, compare, and select the perfect hardware components for your Physical AI projects
            </p>
          </div>
          
          <div className={styles.headerStats}>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>{HardwareList.length}</div>
              <div className={styles.statLabel}>Components</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>{filteredHardware.length}</div>
              <div className={styles.statLabel}>Filtered</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>{selectedHardware.length}</div>
              <div className={styles.statLabel}>Comparing</div>
            </div>
          </div>
        </div>

        {/* Main Content */}
        <div className={styles.mainContent}>
          {/* Sidebar Filters */}
          <div className={clsx(styles.sidebar, isFiltersOpen && styles.open)}>
            <div className={styles.sidebarHeader}>
              <h3 className={styles.sidebarTitle}>
                <FiFilter className={styles.filterIcon} />
                Filters
              </h3>
              <button 
                className={styles.clearFiltersButton}
                onClick={clearFilters}
              >
                Clear All
              </button>
            </div>

            {/* Search */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>Search Hardware</label>
              <div className={styles.searchBox}>
                <input
                  type="text"
                  placeholder="Search by name or description..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className={styles.searchInput}
                />
              </div>
            </div>

            {/* Category Filter */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>Category</label>
              <div className={styles.categoryFilters}>
                {categories.map(category => (
                  <button
                    key={category.id}
                    className={clsx(
                      styles.categoryButton,
                      filter.category === category.id && styles.active
                    )}
                    onClick={() => setFilter(prev => ({ ...prev, category: category.id as HardwareCategory }))}
                    style={{
                      borderColor: category.color,
                      backgroundColor: filter.category === category.id ? category.color : 'transparent',
                      color: filter.category === category.id ? 'white' : category.color
                    }}
                  >
                    {category.icon}
                    {category.name}
                  </button>
                ))}
              </div>
            </div>

            {/* Price Range */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>Price Range</label>
              <div className={styles.priceFilters}>
                {priceRanges.map(range => (
                  <button
                    key={range.id}
                    className={clsx(
                      styles.priceButton,
                      filter.priceRange === range.id && styles.active
                    )}
                    onClick={() => setFilter(prev => ({ ...prev, priceRange: range.id }))}
                  >
                    {range.label}
                  </button>
                ))}
              </div>
            </div>

            {/* Budget Slider */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>
                Max Budget: <span className={styles.budgetValue}>${budget.toLocaleString()}</span>
              </label>
              <input
                type="range"
                min="100"
                max="50000"
                step="100"
                value={budget}
                onChange={(e) => setBudget(parseInt(e.target.value))}
                className={styles.budgetSlider}
              />
              <div className={styles.budgetMarks}>
                <span>$100</span>
                <span>$25,000</span>
                <span>$50,000</span>
              </div>
            </div>

            {/* Energy Efficiency */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>Energy Efficiency</label>
              <div className={styles.efficiencyFilters}>
                {efficiencyOptions.map(efficiency => (
                  <button
                    key={efficiency}
                    className={clsx(
                      styles.efficiencyButton,
                      filter.energyEfficiency.includes(efficiency) && styles.active
                    )}
                    onClick={() => setFilter(prev => ({
                      ...prev,
                      energyEfficiency: prev.energyEfficiency.includes(efficiency)
                        ? prev.energyEfficiency.filter(e => e !== efficiency)
                        : [...prev.energyEfficiency, efficiency]
                    }))}
                    style={{
                      backgroundColor: filter.energyEfficiency.includes(efficiency) 
                        ? getEfficiencyColor(efficiency)
                        : 'transparent',
                      borderColor: getEfficiencyColor(efficiency),
                      color: filter.energyEfficiency.includes(efficiency) ? 'white' : getEfficiencyColor(efficiency)
                    }}
                  >
                    {efficiency}
                  </button>
                ))}
              </div>
            </div>

            {/* Popularity Filter */}
            <div className={styles.filterSection}>
              <label className={styles.filterLabel}>
                Minimum Popularity: <span className={styles.popularityValue}>{filter.popularity}%</span>
              </label>
              <input
                type="range"
                min="0"
                max="100"
                step="5"
                value={filter.popularity}
                onChange={(e) => setFilter(prev => ({ ...prev, popularity: parseInt(e.target.value) }))}
                className={styles.popularitySlider}
              />
            </div>
          </div>

          {/* Main Content Area */}
          <div className={styles.contentArea}>
            {/* View Controls */}
            <div className={styles.viewControls}>
              <div className={styles.viewModeButtons}>
                <button
                  className={clsx(styles.viewModeButton, viewMode === 'grid' && styles.active)}
                  onClick={() => setViewMode('grid')}
                >
                  Grid View
                </button>
                <button
                  className={clsx(styles.viewModeButton, viewMode === 'list' && styles.active)}
                  onClick={() => setViewMode('list')}
                >
                  List View
                </button>
                <button
                  className={clsx(styles.viewModeButton, viewMode === 'comparison' && styles.active)}
                  onClick={() => setViewMode('comparison')}
                >
                  Comparison View
                </button>
              </div>

              <div className={styles.actionButtons}>
                {selectedHardware.length > 0 && (
                  <div className={styles.comparisonBadge}>
                    <FiCompare className={styles.comparisonIcon} />
                    Comparing {selectedHardware.length} items
                    <button 
                      className={styles.clearComparisonButton}
                      onClick={() => setSelectedHardware([])}
                    >
                      Clear
                    </button>
                  </div>
                )}
                <button 
                  className={styles.downloadButton}
                  onClick={() => setIsFiltersOpen(!isFiltersOpen)}
                >
                  {isFiltersOpen ? 'Hide Filters' : 'Show Filters'}
                </button>
              </div>
            </div>

            {/* Comparison Bar */}
            {selectedHardware.length > 0 && (
              <div className={styles.comparisonBar}>
                <div className={styles.comparisonHeader}>
                  <h4 className={styles.comparisonTitle}>
                    <FiCompare className={styles.comparisonIcon} />
                    Compare {selectedHardware.length} Hardware Items
                  </h4>
                  <div className={styles.comparisonStats}>
                    <span className={styles.stat}>
                      Total Cost: <strong>${calculateTotalCost().toLocaleString()}</strong>
                    </span>
                    <button className={styles.exportButton}>
                      <FiDownload className={styles.exportIcon} />
                      Export Comparison
                    </button>
                  </div>
                </div>
                <div className={styles.comparisonItems}>
                  {selectedHardware.map(id => {
                    const item = HardwareList.find(h => h.id === id);
                    if (!item) return null;
                    return (
                      <div key={id} className={styles.comparisonItem}>
                        <span className={styles.itemName}>{item.name}</span>
                        <span className={styles.itemPrice}>${item.cost.min}</span>
                        <button 
                          className={styles.removeButton}
                          onClick={() => handleCompareToggle(id)}
                        >
                          ×
                        </button>
                      </div>
                    );
                  })}
                </div>
              </div>
            )}

            {/* Hardware Grid/List */}
            <div className={clsx(
              styles.hardwareContainer,
              viewMode === 'grid' && styles.gridView,
              viewMode === 'list' && styles.listView,
              viewMode === 'comparison' && styles.comparisonView
            )}>
              {filteredHardware.length === 0 ? (
                <div className={styles.noResults}>
                  <FiFilter className={styles.noResultsIcon} />
                  <h3>No hardware found</h3>
                  <p>Try adjusting your filters or search criteria</p>
                  <button className={styles.resetButton} onClick={clearFilters}>
                    Reset All Filters
                  </button>
                </div>
              ) : (
                filteredHardware.map(hardware => (
                  <HardwareCard
                    key={hardware.id}
                    hardware={hardware}
                    isComparing={selectedHardware.includes(hardware.id)}
                    onCompareToggle={handleCompareToggle}
                  />
                ))
              )}
            </div>

            {/* 3D Model Viewer Placeholder */}
            <div className={styles.modelViewerPlaceholder}>
              <div className={styles.modelViewerHeader}>
                <h4>Interactive 3D Models</h4>
                <p>Rotate, zoom, and explore hardware components in 3D</p>
              </div>
              <div className={styles.modelGrid}>
                <div className={styles.modelItem}>
                  <div className={styles.modelPreview}>
                    <FiRotateCw className={styles.modelIcon} />
                  </div>
                  <span className={styles.modelLabel}>NVIDIA Jetson</span>
                </div>
                <div className={styles.modelItem}>
                  <div className={styles.modelPreview}>
                    <FiCamera className={styles.modelIcon} />
                  </div>
                  <span className={styles.modelLabel}>RealSense Camera</span>
                </div>
                <div className={styles.modelItem}>
                  <div className={styles.modelPreview}>
                    <RiRobot2Line className={styles.modelIcon} />
                  </div>
                  <span className={styles.modelLabel}>Unitree Robot</span>
                </div>
                <div className={styles.modelItem}>
                  <div className={styles.modelPreview}>
                    <FiMonitor className={styles.modelIcon} />
                  </div>
                  <span className={styles.modelLabel}>RTX 4090</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

// Helper function for efficiency colors
const getEfficiencyColor = (rating: string) => {
  switch(rating) {
    case 'A': return '#4A7C59';
    case 'B': return '#8BC34A';
    case 'C': return '#FFC107';
    case 'D': return '#FF9800';
    case 'E': return '#F44336';
    default: return '#9E9E9E';
  }
};

export default HardwareTable;
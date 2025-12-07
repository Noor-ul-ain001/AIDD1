import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './CostCalculator.module.css';

// Import icons
import { 
  FiDollarSign, 
  FiShoppingCart, 
  FiPackage, 
  FiCpu,
  FiMonitor,
  FiServer,
  FiCamera,
  FiRadio,
  FiTool,
  FiDownload,
  FiShare2,
  FiTrendingUp,
  FiCalendar,
  FiPercent,
  FiCreditCard,
  FiSave,
  FiRefreshCw,
  FiChevronDown,
  FiChevronUp,
  FiBarChart2,
  FiPieChart,
  FiGrid,
  FiHelpCircle
} from 'react-icons/fi';
import { RiRobot2Line, RiExchangeDollarLine, RiPriceTagLine } from 'react-icons/ri';

// Types
type LabTier = 'basic' | 'standard' | 'professional' | 'enterprise' | 'custom';
type CostCategory = 'workstation' | 'edge' | 'robot' | 'sensors' | 'accessories' | 'software' | 'maintenance';
type Frequency = 'one-time' | 'monthly' | 'yearly';

interface HardwareOption {
  id: string;
  name: string;
  description: string;
  baseCost: number;
  category: CostCategory;
  frequency: Frequency;
  features: string[];
  recommendedFor: string[];
  optional: boolean;
  quantity: number;
  maxQuantity: number;
  image: string;
  specs?: {
    cpu?: string;
    gpu?: string;
    ram?: string;
    storage?: string;
    power?: string;
  };
}

interface PromoCode {
  code: string;
  discount: number;
  type: 'percentage' | 'fixed';
  minAmount: number;
  validUntil: string;
  categories: CostCategory[];
}

interface CostBreakdown {
  category: CostCategory;
  name: string;
  cost: number;
  quantity: number;
  frequency: Frequency;
}

// Lab Tier Configurations
const labTiers: Record<LabTier, { name: string; description: string; color: string; discount: number }> = {
  basic: {
    name: 'Basic Lab',
    description: 'For students and hobbyists',
    color: '#4A7C59',
    discount: 5
  },
  standard: {
    name: 'Standard Lab',
    description: 'For small research teams',
    color: '#3A6C49',
    discount: 10
  },
  professional: {
    name: 'Professional Lab',
    description: 'For academic research labs',
    color: '#2C4A2C',
    discount: 15
  },
  enterprise: {
    name: 'Enterprise Lab',
    description: 'For corporate R&D',
    color: '#1A3A1A',
    discount: 20
  },
  custom: {
    name: 'Custom Lab',
    description: 'Tailored to your needs',
    color: '#4A7CAA',
    discount: 0
  }
};

// Hardware Options
const hardwareOptions: HardwareOption[] = [
  // Workstations
  {
    id: 'ws-basic',
    name: 'Basic Workstation',
    description: 'Entry-level workstation for basic simulations and coding',
    baseCost: 2500,
    category: 'workstation',
    frequency: 'one-time',
    features: ['NVIDIA RTX 3060', 'Intel i5/AMD Ryzen 5', '32GB RAM', '1TB NVMe SSD'],
    recommendedFor: ['Students', 'Hobbyists', 'Basic ROS development'],
    optional: false,
    quantity: 1,
    maxQuantity: 1,
    image: 'workstation-basic',
    specs: {
      cpu: 'Intel Core i5-13600K / AMD Ryzen 5 7600X',
      gpu: 'NVIDIA RTX 3060 (12GB)',
      ram: '32GB DDR5',
      storage: '1TB NVMe SSD'
    }
  },
  {
    id: 'ws-pro',
    name: 'Professional Workstation',
    description: 'High-performance workstation for AI training and complex simulations',
    baseCost: 5000,
    category: 'workstation',
    frequency: 'one-time',
    features: ['NVIDIA RTX 4090', 'Intel i9/AMD Ryzen 9', '64GB RAM', '2TB NVMe SSD'],
    recommendedFor: ['Researchers', 'Small teams', 'Heavy simulations'],
    optional: false,
    quantity: 0,
    maxQuantity: 1,
    image: 'workstation-pro',
    specs: {
      cpu: 'Intel Core i9-14900K / AMD Ryzen 9 7950X',
      gpu: 'NVIDIA RTX 4090 (24GB)',
      ram: '64GB DDR5',
      storage: '2TB NVMe SSD'
    }
  },
  {
    id: 'ws-ultimate',
    name: 'Ultimate Workstation',
    description: 'Dual GPU workstation for massive AI model training',
    baseCost: 12000,
    category: 'workstation',
    frequency: 'one-time',
    features: ['Dual NVIDIA RTX 4090', 'Intel i9/AMD Threadripper', '128GB RAM', '4TB NVMe SSD'],
    recommendedFor: ['Large research teams', 'Enterprise AI', 'Massive simulations'],
    optional: false,
    quantity: 0,
    maxQuantity: 1,
    image: 'workstation-ultimate',
    specs: {
      cpu: 'AMD Threadripper PRO 7965WX',
      gpu: 'Dual NVIDIA RTX 4090 (48GB total)',
      ram: '128GB DDR5',
      storage: '4TB NVMe SSD'
    }
  },

  // Edge Computing
  {
    id: 'edge-nano',
    name: 'NVIDIA Jetson Nano',
    description: 'Entry-level AI computing for small robots and IoT',
    baseCost: 299,
    category: 'edge',
    frequency: 'one-time',
    features: ['4GB RAM', 'Low power', 'AI ready', 'ROS 2 support'],
    recommendedFor: ['Small robots', 'Educational projects', 'IoT devices'],
    optional: true,
    quantity: 0,
    maxQuantity: 10,
    image: 'jetson-nano'
  },
  {
    id: 'edge-orin-nano',
    name: 'NVIDIA Jetson Orin Nano',
    description: 'Compact AI computing with modern architecture',
    baseCost: 599,
    category: 'edge',
    frequency: 'one-time',
    features: ['8GB RAM', '40 TOPS', 'Low power', 'Robot ready'],
    recommendedFor: ['Autonomous robots', 'Edge AI', 'Smart cameras'],
    optional: true,
    quantity: 0,
    maxQuantity: 10,
    image: 'jetson-orin-nano'
  },
  {
    id: 'edge-orin-agx',
    name: 'NVIDIA Jetson AGX Orin',
    description: 'High-performance AI computing platform',
    baseCost: 1999,
    category: 'edge',
    frequency: 'one-time',
    features: ['32GB RAM', '275 TOPS', 'Multiple sensors', 'Professional SDK'],
    recommendedFor: ['Advanced robots', 'Autonomous vehicles', 'Industrial AI'],
    optional: true,
    quantity: 0,
    maxQuantity: 5,
    image: 'jetson-agx-orin'
  },

  // Robots
  {
    id: 'robot-go1',
    name: 'Unitree Go1 Edu',
    description: 'Educational quadruped robot for research',
    baseCost: 12000,
    category: 'robot',
    frequency: 'one-time',
    features: ['Dynamic locomotion', 'ROS 2 support', 'Multiple sensors', '3 hour battery'],
    recommendedFor: ['Academic research', 'Robotics courses', 'Prototyping'],
    optional: true,
    quantity: 0,
    maxQuantity: 3,
    image: 'unitree-go1'
  },
  {
    id: 'robot-go2',
    name: 'Unitree Go2 Pro',
    description: 'Advanced quadruped robot with enhanced capabilities',
    baseCost: 30000,
    category: 'robot',
    frequency: 'one-time',
    features: ['Advanced locomotion', '360° perception', 'Payload: 5kg', 'Professional SDK'],
    recommendedFor: ['Research labs', 'Corporate R&D', 'Advanced projects'],
    optional: true,
    quantity: 0,
    maxQuantity: 2,
    image: 'unitree-go2'
  },
  {
    id: 'robot-humanoid',
    name: 'Advanced Humanoid',
    description: 'Research-grade humanoid robot platform',
    baseCost: 85000,
    category: 'robot',
    frequency: 'one-time',
    features: ['Full humanoid design', 'Advanced AI integration', 'Research SDK', 'Professional support'],
    recommendedFor: ['Advanced research', 'Corporate innovation', 'Future robotics'],
    optional: true,
    quantity: 0,
    maxQuantity: 1,
    image: 'robot-humanoid'
  },

  // Sensors
  {
    id: 'sensor-realsense',
    name: 'Intel RealSense D455',
    description: 'Depth camera for 3D perception and SLAM',
    baseCost: 499,
    category: 'sensors',
    frequency: 'one-time',
    features: ['RGB-D camera', 'IMU', 'Global shutter', 'ROS 2 drivers'],
    recommendedFor: ['3D perception', 'SLAM', 'Object detection'],
    optional: true,
    quantity: 0,
    maxQuantity: 5,
    image: 'realsense-d455'
  },
  {
    id: 'sensor-lidar',
    name: '3D LiDAR',
    description: 'High-precision 3D LiDAR for mapping',
    baseCost: 2500,
    category: 'sensors',
    frequency: 'one-time',
    features: ['360° scanning', 'High precision', 'Long range', 'ROS integration'],
    recommendedFor: ['Autonomous navigation', '3D mapping', 'Environmental sensing'],
    optional: true,
    quantity: 0,
    maxQuantity: 2,
    image: 'lidar-3d'
  },
  {
    id: 'sensor-imu',
    name: 'High-Precision IMU',
    description: 'Inertial measurement unit for navigation',
    baseCost: 299,
    category: 'sensors',
    frequency: 'one-time',
    features: ['9-axis IMU', 'High accuracy', 'Low drift', 'Kalman filtering'],
    recommendedFor: ['Navigation', 'State estimation', 'Motion tracking'],
    optional: true,
    quantity: 0,
    maxQuantity: 5,
    image: 'imu-high-precision'
  },

  // Accessories
  {
    id: 'accessory-charging',
    name: 'Robot Charging Station',
    description: 'Automatic charging station for robots',
    baseCost: 1200,
    category: 'accessories',
    frequency: 'one-time',
    features: ['Automatic docking', 'Fast charging', 'Multiple robots', 'Safety features'],
    recommendedFor: ['Robot fleets', 'Continuous operation', 'Automation'],
    optional: true,
    quantity: 0,
    maxQuantity: 3,
    image: 'charging-station'
  },
  {
    id: 'accessory-tools',
    name: 'Robotics Tool Kit',
    description: 'Complete toolkit for robot maintenance',
    baseCost: 299,
    category: 'accessories',
    frequency: 'one-time',
    features: ['Professional tools', 'Calibration tools', 'Safety equipment', 'Carrying case'],
    recommendedFor: ['Maintenance', 'Calibration', 'Repairs'],
    optional: true,
    quantity: 0,
    maxQuantity: 2,
    image: 'tool-kit'
  },

  // Software
  {
    id: 'software-ros',
    name: 'ROS 2 Enterprise',
    description: 'Professional ROS 2 distribution with support',
    baseCost: 1000,
    category: 'software',
    frequency: 'yearly',
    features: ['Professional support', 'Security updates', 'Long-term support', 'Priority bug fixes'],
    recommendedFor: ['Enterprise', 'Production', 'Critical systems'],
    optional: true,
    quantity: 0,
    maxQuantity: 5,
    image: 'ros-enterprise'
  },
  {
    id: 'software-sim',
    name: 'Simulation Suite',
    description: 'Advanced simulation software',
    baseCost: 2500,
    category: 'software',
    frequency: 'yearly',
    features: ['Physics simulation', 'Multi-robot', 'Sensor simulation', 'Cloud rendering'],
    recommendedFor: ['Large simulations', 'Multi-agent', 'Complex environments'],
    optional: true,
    quantity: 0,
    maxQuantity: 2,
    image: 'simulation-suite'
  },

  // Maintenance
  {
    id: 'maintenance-basic',
    name: 'Basic Maintenance',
    description: 'Standard maintenance and support',
    baseCost: 1000,
    category: 'maintenance',
    frequency: 'yearly',
    features: ['Remote support', 'Software updates', 'Basic troubleshooting', 'Email support'],
    recommendedFor: ['Small labs', 'Educational use', 'Basic needs'],
    optional: true,
    quantity: 0,
    maxQuantity: 1,
    image: 'maintenance-basic'
  },
  {
    id: 'maintenance-premium',
    name: 'Premium Support',
    description: 'Premium maintenance and 24/7 support',
    baseCost: 5000,
    category: 'maintenance',
    frequency: 'yearly',
    features: ['24/7 support', 'On-site visits', 'Priority service', 'Extended warranty'],
    recommendedFor: ['Enterprise', 'Production', 'Critical systems'],
    optional: true,
    quantity: 0,
    maxQuantity: 1,
    image: 'maintenance-premium'
  }
];

// Promo Codes
const promoCodes: PromoCode[] = [
  { code: 'STUDENT15', discount: 15, type: 'percentage', minAmount: 1000, validUntil: '2024-12-31', categories: ['workstation', 'edge', 'robot'] },
  { code: 'RESEARCH20', discount: 20, type: 'percentage', minAmount: 5000, validUntil: '2024-12-31', categories: ['workstation', 'robot'] },
  { code: 'EDUCATION500', discount: 500, type: 'fixed', minAmount: 3000, validUntil: '2024-12-31', categories: ['workstation', 'edge'] },
  { code: 'WELCOME10', discount: 10, type: 'percentage', minAmount: 1000, validUntil: '2024-12-31', categories: [] }
];

// Preset Configurations
const presetConfigs = {
  basic: ['ws-basic'],
  standard: ['ws-basic', 'edge-nano', 'sensor-realsense'],
  professional: ['ws-pro', 'edge-orin-nano', 'robot-go1', 'sensor-realsense', 'sensor-imu'],
  enterprise: ['ws-ultimate', 'edge-orin-agx', 'robot-go2', 'sensor-realsense', 'sensor-lidar', 'software-ros', 'maintenance-premium']
};

const CostCalculator = () => {
  // State Management
  const [selectedTier, setSelectedTier] = useState<LabTier>('standard');
  const [selectedItems, setSelectedItems] = useState<HardwareOption[]>(() => {
    const presetIds = presetConfigs[selectedTier];
    return hardwareOptions.filter(item => presetIds.includes(item.id)).map(item => ({
      ...item,
      quantity: item.quantity === 0 ? 1 : item.quantity
    }));
  });
  const [promoCode, setPromoCode] = useState('');
  const [appliedPromo, setAppliedPromo] = useState<PromoCode | null>(null);
  const [promoError, setPromoError] = useState('');
  const [currency, setCurrency] = useState('USD');
  const [exchangeRate, setExchangeRate] = useState(1);
  const [viewMode, setViewMode] = useState<'list' | 'chart'>('chart');
  const [expandedCategory, setExpandedCategory] = useState<CostCategory | null>('workstation');
  const [budget, setBudget] = useState<number>(10000);
  const [timeframe, setTimeframe] = useState<'one-time' | 'yearly'>('one-time');

  // Currency Options
  const currencies = [
    { code: 'USD', symbol: '$', name: 'US Dollar' },
    { code: 'EUR', symbol: '€', name: 'Euro' },
    { code: 'GBP', symbol: '£', name: 'British Pound' },
    { code: 'JPY', symbol: '¥', name: 'Japanese Yen' },
    { code: 'CAD', symbol: 'C$', name: 'Canadian Dollar' },
    { code: 'AUD', symbol: 'A$', name: 'Australian Dollar' }
  ];

  // Calculate Costs
  const calculateCosts = () => {
    let subtotal = selectedItems.reduce((total, item) => {
      return total + (item.baseCost * item.quantity);
    }, 0);

    // Apply tier discount
    const tierDiscount = labTiers[selectedTier].discount;
    const discountAmount = subtotal * (tierDiscount / 100);

    // Apply promo discount
    let promoDiscount = 0;
    if (appliedPromo) {
      if (appliedPromo.type === 'percentage') {
        promoDiscount = subtotal * (appliedPromo.discount / 100);
      } else {
        promoDiscount = appliedPromo.discount;
      }
    }

    const totalDiscount = discountAmount + promoDiscount;
    const totalAfterDiscount = Math.max(0, subtotal - totalDiscount);
    
    // Add taxes (estimate 8% for most regions)
    const taxRate = 0.08;
    const taxAmount = totalAfterDiscount * taxRate;
    
    const total = totalAfterDiscount + taxAmount;

    // Yearly cost projection
    const yearlyRecurring = selectedItems
      .filter(item => item.frequency === 'yearly')
      .reduce((total, item) => total + (item.baseCost * item.quantity), 0);

    return {
      subtotal,
      tierDiscount: discountAmount,
      promoDiscount,
      totalDiscount,
      taxAmount,
      total,
      yearlyRecurring,
      totalYearly: total + yearlyRecurring
    };
  };

  const costs = calculateCosts();

  // Handle Item Selection
  const handleItemToggle = (itemId: string) => {
    const item = hardwareOptions.find(h => h.id === itemId);
    if (!item) return;

    const isSelected = selectedItems.some(selected => selected.id === itemId);
    
    if (isSelected) {
      setSelectedItems(prev => prev.filter(i => i.id !== itemId));
    } else {
      setSelectedItems(prev => [...prev, { ...item, quantity: 1 }]);
    }
  };

  const handleQuantityChange = (itemId: string, quantity: number) => {
    setSelectedItems(prev => prev.map(item => 
      item.id === itemId 
        ? { ...item, quantity: Math.min(Math.max(quantity, 0), item.maxQuantity) }
        : item
    ));
  };

  // Apply Preset
  const applyPreset = (tier: LabTier) => {
    setSelectedTier(tier);
    const presetIds = presetConfigs[tier];
    const presetItems = hardwareOptions
      .filter(item => presetIds.includes(item.id))
      .map(item => ({ ...item, quantity: 1 }));
    setSelectedItems(presetItems);
  };

  // Apply Promo Code
  const handleApplyPromo = () => {
    setPromoError('');
    
    if (!promoCode.trim()) {
      setPromoError('Please enter a promo code');
      return;
    }

    const promo = promoCodes.find(p => p.code === promoCode.toUpperCase());
    
    if (!promo) {
      setPromoError('Invalid promo code');
      return;
    }

    if (new Date(promo.validUntil) < new Date()) {
      setPromoError('Promo code has expired');
      return;
    }

    if (costs.subtotal < promo.minAmount) {
      setPromoError(`Minimum purchase of $${promo.minAmount} required`);
      return;
    }

    if (promo.categories.length > 0) {
      const hasEligibleItems = selectedItems.some(item => 
        promo.categories.includes(item.category)
      );
      if (!hasEligibleItems) {
        setPromoError('Promo code not valid for selected items');
        return;
      }
    }

    setAppliedPromo(promo);
  };

  // Export Configuration
  const exportConfiguration = () => {
    const config = {
      tier: selectedTier,
      items: selectedItems.map(item => ({
        id: item.id,
        name: item.name,
        quantity: item.quantity,
        cost: item.baseCost * item.quantity
      })),
      costs,
      timestamp: new Date().toISOString()
    };

    const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `lab-configuration-${new Date().toISOString().split('T')[0]}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  // Share Configuration
  const shareConfiguration = () => {
    const shareText = `Check out my Physical AI Lab configuration! Total cost: $${costs.total.toLocaleString()}`;
    const shareUrl = window.location.href;
    
    if (navigator.share) {
      navigator.share({
        title: 'My Lab Configuration',
        text: shareText,
        url: shareUrl
      });
    } else {
      navigator.clipboard.writeText(`${shareText}\n${shareUrl}`);
      alert('Configuration link copied to clipboard!');
    }
  };

  // Get category total
  const getCategoryTotal = (category: CostCategory) => {
    return selectedItems
      .filter(item => item.category === category)
      .reduce((total, item) => total + (item.baseCost * item.quantity), 0);
  };

  // Get category icon
  const getCategoryIcon = (category: CostCategory) => {
    switch(category) {
      case 'workstation': return <FiMonitor />;
      case 'edge': return <FiCpu />;
      case 'robot': return <RiRobot2Line />;
      case 'sensors': return <FiCamera />;
      case 'accessories': return <FiTool />;
      case 'software': return <FiServer />;
      case 'maintenance': return <FiPackage />;
      default: return <FiPackage />;
    }
  };

  // Get category color
  const getCategoryColor = (category: CostCategory) => {
    switch(category) {
      case 'workstation': return '#4A7C59';
      case 'edge': return '#3A6C49';
      case 'robot': return '#2C4A2C';
      case 'sensors': return '#4A7CAA';
      case 'accessories': return '#D87C4A';
      case 'software': return '#8A4C8C';
      case 'maintenance': return '#4A7C59';
      default: return '#4A7C59';
    }
  };

  // Get category name
  const getCategoryName = (category: CostCategory) => {
    switch(category) {
      case 'workstation': return 'Workstations';
      case 'edge': return 'Edge Computing';
      case 'robot': return 'Robots';
      case 'sensors': return 'Sensors';
      case 'accessories': return 'Accessories';
      case 'software': return 'Software';
      case 'maintenance': return 'Maintenance';
      default: return 'Other';
    }
  };

  // Group items by category
  const itemsByCategory = hardwareOptions.reduce((acc, item) => {
    if (!acc[item.category]) {
      acc[item.category] = [];
    }
    acc[item.category].push(item);
    return acc;
  }, {} as Record<CostCategory, HardwareOption[]>);

  const categories = Object.keys(itemsByCategory) as CostCategory[];

  // Budget progress
  const budgetPercentage = Math.min(100, (costs.total / budget) * 100);
  const isOverBudget = costs.total > budget;

  return (
    <section className={styles.costCalculator}>
      <div className="container">
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerContent}>
            <h2 className={styles.title}>
              <RiExchangeDollarLine className={styles.titleIcon} />
              Lab Setup Cost Calculator
            </h2>
            <p className={styles.subtitle}>
              Build and price your perfect Physical AI laboratory. Compare configurations, apply discounts, and plan your budget.
            </p>
          </div>
          <div className={styles.headerStats}>
            <div className={styles.stat}>
              <div className={styles.statNumber}>${budget.toLocaleString()}</div>
              <div className={styles.statLabel}>Your Budget</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>${costs.total.toLocaleString()}</div>
              <div className={styles.statLabel}>Total Cost</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>
                {Math.round((budget - costs.total) / budget * 100)}%
              </div>
              <div className={styles.statLabel}>Remaining</div>
            </div>
          </div>
        </div>

        <div className={styles.mainContent}>
          {/* Left Column: Configuration */}
          <div className={styles.configurationPanel}>
            {/* Tier Selection */}
            <div className={styles.tierSelection}>
              <h3 className={styles.panelTitle}>Select Lab Tier</h3>
              <div className={styles.tierButtons}>
                {Object.entries(labTiers).map(([tier, config]) => (
                  <button
                    key={tier}
                    className={clsx(
                      styles.tierButton,
                      selectedTier === tier && styles.selected
                    )}
                    onClick={() => applyPreset(tier as LabTier)}
                    style={{
                      borderColor: config.color,
                      backgroundColor: selectedTier === tier ? config.color : 'transparent',
                      color: selectedTier === tier ? 'white' : config.color
                    }}
                  >
                    <span className={styles.tierName}>{config.name}</span>
                    <span className={styles.tierDescription}>{config.description}</span>
                    <span className={styles.tierDiscount}>{config.discount}% OFF</span>
                  </button>
                ))}
              </div>
            </div>

            {/* Budget Control */}
            <div className={styles.budgetControl}>
              <h3 className={styles.panelTitle}>
                <FiTrendingUp className={styles.panelIcon} />
                Budget Control
              </h3>
              <div className={styles.budgetSliderContainer}>
                <div className={styles.budgetLabels}>
                  <span className={styles.budgetLabel}>Set Budget:</span>
                  <span className={styles.budgetValue}>${budget.toLocaleString()}</span>
                </div>
                <input
                  type="range"
                  min="1000"
                  max="100000"
                  step="1000"
                  value={budget}
                  onChange={(e) => setBudget(parseInt(e.target.value))}
                  className={styles.budgetSlider}
                />
                <div className={styles.budgetMarks}>
                  <span>$1K</span>
                  <span>$50K</span>
                  <span>$100K</span>
                </div>
              </div>

              {/* Budget Progress */}
              <div className={styles.budgetProgress}>
                <div className={styles.progressLabels}>
                  <span>Budget Usage</span>
                  <span className={clsx(styles.progressPercentage, isOverBudget && styles.overBudget)}>
                    {budgetPercentage.toFixed(1)}%
                  </span>
                </div>
                <div className={styles.progressBar}>
                  <div 
                    className={clsx(styles.progressFill, isOverBudget && styles.overBudget)}
                    style={{ width: `${Math.min(100, budgetPercentage)}%` }}
                  />
                </div>
                <div className={styles.progressInfo}>
                  {isOverBudget ? (
                    <span className={styles.overBudgetWarning}>
                      ⚠️ ${(costs.total - budget).toLocaleString()} over budget
                    </span>
                  ) : (
                    <span className={styles.underBudget}>
                      💰 ${(budget - costs.total).toLocaleString()} remaining
                    </span>
                  )}
                </div>
              </div>
            </div>

            {/* Categories */}
            <div className={styles.categoriesPanel}>
              <h3 className={styles.panelTitle}>
                <FiGrid className={styles.panelIcon} />
                Hardware & Software
              </h3>
              <div className={styles.categoriesList}>
                {categories.map(category => (
                  <div key={category} className={styles.categorySection}>
                    <button
                      className={styles.categoryHeader}
                      onClick={() => setExpandedCategory(expandedCategory === category ? null : category)}
                    >
                      <div className={styles.categoryInfo}>
                        <div className={styles.categoryIcon} style={{ color: getCategoryColor(category) }}>
                          {getCategoryIcon(category)}
                        </div>
                        <div>
                          <h4 className={styles.categoryName}>{getCategoryName(category)}</h4>
                          <div className={styles.categoryStats}>
                            <span className={styles.categoryCost}>
                              ${getCategoryTotal(category).toLocaleString()}
                            </span>
                            <span className={styles.categoryItems}>
                              {selectedItems.filter(i => i.category === category).length} items
                            </span>
                          </div>
                        </div>
                      </div>
                      {expandedCategory === category ? (
                        <FiChevronUp className={styles.expandIcon} />
                      ) : (
                        <FiChevronDown className={styles.expandIcon} />
                      )}
                    </button>

                    {expandedCategory === category && (
                      <div className={styles.categoryContent}>
                        <div className={styles.itemsList}>
                          {itemsByCategory[category].map(item => {
                            const isSelected = selectedItems.some(selected => selected.id === item.id);
                            const selectedItem = selectedItems.find(selected => selected.id === item.id);
                            
                            return (
                              <div key={item.id} className={clsx(styles.itemCard, isSelected && styles.selected)}>
                                <div className={styles.itemHeader}>
                                  <div className={styles.itemInfo}>
                                    <h5 className={styles.itemName}>{item.name}</h5>
                                    <p className={styles.itemDescription}>{item.description}</p>
                                    <div className={styles.itemFeatures}>
                                      {item.features.slice(0, 2).map((feature, idx) => (
                                        <span key={idx} className={styles.featureTag}>
                                          {feature}
                                        </span>
                                      ))}
                                    </div>
                                  </div>
                                  <div className={styles.itemCost}>
                                    <div className={styles.costAmount}>
                                      ${item.baseCost.toLocaleString()}
                                    </div>
                                    <div className={styles.costFrequency}>
                                      {item.frequency === 'one-time' ? 'one-time' : '/year'}
                                    </div>
                                  </div>
                                </div>

                                <div className={styles.itemActions}>
                                  <div className={styles.recommendedFor}>
                                    <span className={styles.recommendedLabel}>Best for:</span>
                                    <span>{item.recommendedFor.join(', ')}</span>
                                  </div>
                                  
                                  {isSelected ? (
                                    <div className={styles.quantityControl}>
                                      <button
                                        className={styles.quantityButton}
                                        onClick={() => handleQuantityChange(item.id, selectedItem.quantity - 1)}
                                        disabled={selectedItem.quantity <= 1}
                                      >
                                        −
                                      </button>
                                      <span className={styles.quantityValue}>
                                        {selectedItem.quantity} unit{selectedItem.quantity !== 1 ? 's' : ''}
                                      </span>
                                      <button
                                        className={styles.quantityButton}
                                        onClick={() => handleQuantityChange(item.id, selectedItem.quantity + 1)}
                                        disabled={selectedItem.quantity >= item.maxQuantity}
                                      >
                                        +
                                      </button>
                                      <button
                                        className={styles.removeButton}
                                        onClick={() => handleItemToggle(item.id)}
                                      >
                                        Remove
                                      </button>
                                    </div>
                                  ) : (
                                    <button
                                      className={styles.addButton}
                                      onClick={() => handleItemToggle(item.id)}
                                      disabled={!item.optional && item.quantity === 0}
                                    >
                                      Add to Lab
                                    </button>
                                  )}
                                </div>
                              </div>
                            );
                          })}
                        </div>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* Right Column: Summary & Controls */}
          <div className={styles.summaryPanel}>
            {/* Cost Summary */}
            <div className={styles.costSummary}>
              <h3 className={styles.panelTitle}>
                <FiDollarSign className={styles.panelIcon} />
                Cost Summary
              </h3>
              
              {/* Timeframe Toggle */}
              <div className={styles.timeframeToggle}>
                <button
                  className={clsx(styles.timeframeButton, timeframe === 'one-time' && styles.active)}
                  onClick={() => setTimeframe('one-time')}
                >
                  One-Time Cost
                </button>
                <button
                  className={clsx(styles.timeframeButton, timeframe === 'yearly' && styles.active)}
                  onClick={() => setTimeframe('yearly')}
                >
                  Yearly Total
                </button>
              </div>

              {/* Cost Breakdown */}
              <div className={styles.breakdown}>
                <div className={styles.breakdownRow}>
                  <span className={styles.breakdownLabel}>Subtotal</span>
                  <span className={styles.breakdownValue}>${costs.subtotal.toLocaleString()}</span>
                </div>
                
                {costs.tierDiscount > 0 && (
                  <div className={styles.breakdownRow}>
                    <span className={styles.breakdownLabel}>
                      Tier Discount ({labTiers[selectedTier].discount}%)
                    </span>
                    <span className={styles.breakdownValue} style={{ color: '#4A7C59' }}>
                      -${costs.tierDiscount.toLocaleString()}
                    </span>
                  </div>
                )}
                
                {costs.promoDiscount > 0 && appliedPromo && (
                  <div className={styles.breakdownRow}>
                    <span className={styles.breakdownLabel}>
                      Promo ({appliedPromo.code})
                    </span>
                    <span className={styles.breakdownValue} style={{ color: '#4A7C59' }}>
                      -${costs.promoDiscount.toLocaleString()}
                    </span>
                  </div>
                )}
                
                <div className={styles.breakdownRow}>
                  <span className={styles.breakdownLabel}>Tax (8% est.)</span>
                  <span className={styles.breakdownValue}>${costs.taxAmount.toLocaleString()}</span>
                </div>
                
                {timeframe === 'yearly' && costs.yearlyRecurring > 0 && (
                  <div className={styles.breakdownRow}>
                    <span className={styles.breakdownLabel}>Yearly Recurring</span>
                    <span className={styles.breakdownValue}>+${costs.yearlyRecurring.toLocaleString()}</span>
                  </div>
                )}
                
                <div className={styles.breakdownDivider} />
                
                <div className={clsx(styles.breakdownRow, styles.totalRow)}>
                  <span className={styles.totalLabel}>
                    {timeframe === 'one-time' ? 'Total Cost' : 'Total Yearly Cost'}
                  </span>
                  <span className={styles.totalValue}>
                    ${(timeframe === 'one-time' ? costs.total : costs.totalYearly).toLocaleString()}
                  </span>
                </div>
              </div>

              {/* Cost Visualization */}
              <div className={styles.visualization}>
                <div className={styles.visualizationHeader}>
                  <h4>Cost Distribution</h4>
                  <div className={styles.viewToggle}>
                    <button
                      className={clsx(styles.viewButton, viewMode === 'chart' && styles.active)}
                      onClick={() => setViewMode('chart')}
                    >
                      <FiPieChart />
                    </button>
                    <button
                      className={clsx(styles.viewButton, viewMode === 'list' && styles.active)}
                      onClick={() => setViewMode('list')}
                    >
                      <FiBarChart2 />
                    </button>
                  </div>
                </div>

                {viewMode === 'chart' ? (
                  <div className={styles.pieChart}>
                    <div className={styles.pieChartVisual}>
                      {categories.map((category, index) => {
                        const categoryTotal = getCategoryTotal(category);
                        if (categoryTotal === 0) return null;
                        
                        const percentage = (categoryTotal / costs.subtotal) * 100;
                        const rotation = categories
                          .slice(0, index)
                          .reduce((sum, cat) => sum + (getCategoryTotal(cat) / costs.subtotal) * 360, 0);
                        
                        return (
                          <div
                            key={category}
                            className={styles.pieSegment}
                            style={{
                              backgroundColor: getCategoryColor(category),
                              transform: `rotate(${rotation}deg)`,
                              clipPath: `conic-gradient(transparent 0deg, transparent ${percentage}%, ${getCategoryColor(category)} ${percentage}%)`
                            }}
                          />
                        );
                      })}
                    </div>
                    <div className={styles.pieLegend}>
                      {categories.map(category => {
                        const categoryTotal = getCategoryTotal(category);
                        if (categoryTotal === 0) return null;
                        
                        return (
                          <div key={category} className={styles.legendItem}>
                            <div className={styles.legendColor} style={{ backgroundColor: getCategoryColor(category) }} />
                            <span className={styles.legendLabel}>{getCategoryName(category)}</span>
                            <span className={styles.legendValue}>${categoryTotal.toLocaleString()}</span>
                          </div>
                        );
                      })}
                    </div>
                  </div>
                ) : (
                  <div className={styles.barChart}>
                    {categories.map(category => {
                      const categoryTotal = getCategoryTotal(category);
                      if (categoryTotal === 0) return null;
                      
                      const percentage = (categoryTotal / costs.subtotal) * 100;
                      
                      return (
                        <div key={category} className={styles.barItem}>
                          <div className={styles.barLabel}>
                            <span>{getCategoryName(category)}</span>
                            <span>${categoryTotal.toLocaleString()}</span>
                          </div>
                          <div className={styles.barContainer}>
                            <div 
                              className={styles.barFill}
                              style={{ 
                                width: `${percentage}%`,
                                backgroundColor: getCategoryColor(category)
                              }}
                            />
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            </div>

            {/* Promo Code */}
            <div className={styles.promoSection}>
              <h4 className={styles.promoTitle}>
                <RiPriceTagLine className={styles.promoIcon} />
                Promo Code
              </h4>
              <div className={styles.promoInputGroup}>
                <input
                  type="text"
                  placeholder="Enter promo code"
                  value={promoCode}
                  onChange={(e) => setPromoCode(e.target.value)}
                  className={styles.promoInput}
                  disabled={!!appliedPromo}
                />
                <button
                  className={styles.promoButton}
                  onClick={handleApplyPromo}
                  disabled={!!appliedPromo}
                >
                  {appliedPromo ? 'Applied' : 'Apply'}
                </button>
              </div>
              {promoError && (
                <div className={styles.promoError}>{promoError}</div>
              )}
              {appliedPromo && (
                <div className={styles.promoSuccess}>
                  ✅ {appliedPromo.code} applied: -${costs.promoDiscount.toLocaleString()}
                  <button 
                    className={styles.removePromoButton}
                    onClick={() => {
                      setAppliedPromo(null);
                      setPromoCode('');
                    }}
                  >
                    Remove
                  </button>
                </div>
              )}
              <div className={styles.availablePromos}>
                <span className={styles.promoHint}>Available codes: STUDENT15, RESEARCH20, EDUCATION500</span>
              </div>
            </div>

            {/* Currency Selection */}
            <div className={styles.currencySection}>
              <h4 className={styles.currencyTitle}>
                <FiCreditCard className={styles.currencyIcon} />
                Currency
              </h4>
              <select 
                className={styles.currencySelect}
                value={currency}
                onChange={(e) => setCurrency(e.target.value)}
              >
                {currencies.map(curr => (
                  <option key={curr.code} value={curr.code}>
                    {curr.symbol} {curr.name} ({curr.code})
                  </option>
                ))}
              </select>
              <div className={styles.exchangeNote}>
                * Prices converted at current exchange rates
              </div>
            </div>

            {/* Action Buttons */}
            <div className={styles.actionButtons}>
              <button className={styles.saveButton} onClick={exportConfiguration}>
                <FiSave className={styles.buttonIcon} />
                Save Configuration
              </button>
              <button className={styles.shareButton} onClick={shareConfiguration}>
                <FiShare2 className={styles.buttonIcon} />
                Share
              </button>
              <button 
                className={styles.resetButton}
                onClick={() => applyPreset('basic')}
              >
                <FiRefreshCw className={styles.buttonIcon} />
                Reset
              </button>
              <button className={styles.exportButton} onClick={exportConfiguration}>
                <FiDownload className={styles.buttonIcon} />
                Export
              </button>
            </div>

            {/* Quote Request */}
            <div className={styles.quoteSection}>
              <div className={styles.quoteContent}>
                <h4 className={styles.quoteTitle}>Need a custom quote?</h4>
                <p className={styles.quoteDescription}>
                  Contact us for volume discounts, custom configurations, or academic pricing.
                </p>
                <button className={styles.quoteButton}>
                  Request Custom Quote
                </button>
              </div>
            </div>

            {/* Help & Tips */}
            <div className={styles.helpSection}>
              <div className={styles.helpHeader}>
                <FiHelpCircle className={styles.helpIcon} />
                <h4>Tips for Planning</h4>
              </div>
              <ul className={styles.tipsList}>
                <li>Start with the Standard tier and add components as needed</li>
                <li>Consider yearly recurring costs for software and maintenance</li>
                <li>Student discounts available for eligible institutions</li>
                <li>Bulk discounts available for multiple units</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default CostCalculator;
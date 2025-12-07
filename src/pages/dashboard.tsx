import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import styles from './dashboard.module.css'; // Assuming a CSS module for styling
import axios from 'axios';

interface UserProfile {
  email: string;
  software_background: string;
  hardware_experience: string;
  preferred_learning: string;
}

const Dashboard = () => {
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [message, setMessage] = useState('');
  const [ingestionStatus, setIngestionStatus] = useState('');

  useEffect(() => {
    const fetchUserProfile = async () => {
      const accessToken = localStorage.getItem('accessToken');
      if (!accessToken) {
        setMessage('You are not logged in. Please log in to view your dashboard.');
        return;
      }

      try {
        const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
        // For now, we will mock the user profile data.
        setUserProfile({
          email: 'user@example.com', // Mock data
          software_background: 'Intermediate',
          hardware_experience: 'ROS',
          preferred_learning: 'Code-heavy',
        });
        setMessage('User profile loaded (mock data).');
      } catch (error) {
        console.error('Error fetching user profile:', error);
        setMessage('Failed to load user profile. Please try again later.');
      }
    };

    fetchUserProfile();
  }, []);

  const handleLogout = () => {
    localStorage.removeItem('accessToken');
    setMessage('Logged out successfully. Redirecting...');
    window.location.href = '/'; // Redirect to home page
  };

  const handleIngestDocuments = async () => {
    setIngestionStatus('Ingesting documents... This may take a moment.');
    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      const response = await axios.post(`${backendUrl}/ingestion/ingest`);
      setIngestionStatus(`Ingestion successful: ${response.data.message}`);
    } catch (error) {
      console.error('Error ingesting documents:', error);
      setIngestionStatus(`Ingestion failed: ${error.response?.data?.detail || error.message}`);
    }
  };

  return (
    <Layout title="User Dashboard" description="View and manage your profile">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">User Dashboard</h1>
          <p className="hero__subtitle">Your personalized learning hub</p>
        </div>
      </header>
      <main className={clsx(styles.dashboardContainer)}>
        <div className="container">
          {userProfile ? (
            <div className={styles.profileCard}>
              <h3>Welcome, {userProfile.email}!</h3>
              <p><strong>Software Background:</strong> {userProfile.software_background}</p>
              <p><strong>Hardware Experience:</strong> {userProfile.hardware_experience}</p>
              <p><strong>Preferred Learning Style:</strong> {userProfile.preferred_learning}</p>
              
              <div className={styles.dashboardActions}>
                <button onClick={handleIngestDocuments} className="button button--primary">
                  Ingest Documents
                </button>
                {ingestionStatus && <p className={styles.ingestionStatus}>{ingestionStatus}</p>}
                <button onClick={handleLogout} className="button button--danger">Logout</button>
              </div>
            </div>
          ) : (
            <p>{message}</p>
          )}
        </div>
      </main>
    </Layout>
  );
};

export default Dashboard;

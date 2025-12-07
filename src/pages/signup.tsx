import React, { useState, FormEvent, ChangeEvent } from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import styles from './signup.module.css'; // Assuming a CSS module for styling
import axios from 'axios';

const Signup = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<'Beginner' | 'Intermediate' | 'Advanced'>('Beginner');
  const [hardwareExperience, setHardwareExperience] = useState<'None' | 'Arduino/RPi' | 'ROS' | 'Robotics Kit'>('None');
  const [preferredLearning, setPreferredLearning] = useState<'Visual' | 'Code-heavy' | 'Theory' | 'Hands-on'>('Visual');
  const [message, setMessage] = useState('');

  const handleSignup = async (event: FormEvent) => {
    event.preventDefault();
    setMessage('');

    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      const response = await axios.post(`${backendUrl}/auth/signup`, {
        email,
        password,
        software_background: softwareBackground,
        hardware_experience: hardwareExperience,
        preferred_learning: preferredLearning,
      });

      // Assuming successful signup returns a token
      localStorage.setItem('accessToken', response.data.access_token);
      setMessage('Signup successful! Redirecting...');
      // In a real app, you would redirect the user to a dashboard or home page
      window.location.href = '/docs/intro'; 
    } catch (error) {
      console.error('Signup error:', error);
      if (axios.isAxiosError(error) && error.response) {
        setMessage(`Signup failed: ${error.response.data.detail || 'Unknown error'}`);
      } else {
        setMessage('Signup failed: An unexpected error occurred.');
      }
    }
  };

  return (
    <Layout title="Sign Up" description="Sign up for the AI-Native Textbook">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">Sign Up</h1>
          <p className="hero__subtitle">Create your account to personalize your learning experience</p>
        </div>
      </header>
      <main className={clsx(styles.signupContainer)}>
        <div className="container">
          <form onSubmit={handleSignup} className={styles.signupForm}>
            <div className={styles.formGroup}>
              <label htmlFor="email">Email:</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e: ChangeEvent<HTMLInputElement>) => setEmail(e.target.value)}
                required
              />
            </div>
            <div className={styles.formGroup}>
              <label htmlFor="password">Password:</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e: ChangeEvent<HTMLInputElement>) => setPassword(e.target.value)}
                required
              />
            </div>
            <div className={styles.formGroup}>
              <label htmlFor="softwareBackground">Software Background:</label>
              <select
                id="softwareBackground"
                value={softwareBackground}
                onChange={(e: ChangeEvent<HTMLSelectElement>) => setSoftwareBackground(e.target.value as 'Beginner' | 'Intermediate' | 'Advanced')}
              >
                <option value="Beginner">Beginner</option>
                <option value="Intermediate">Intermediate</option>
                <option value="Advanced">Advanced</option>
              </select>
            </div>
            <div className={styles.formGroup}>
              <label htmlFor="hardwareExperience">Hardware Experience:</label>
              <select
                id="hardwareExperience"
                value={hardwareExperience}
                onChange={(e: ChangeEvent<HTMLSelectElement>) => setHardwareExperience(e.target.value as 'None' | 'Arduino/RPi' | 'ROS' | 'Robotics Kit')}
              >
                <option value="None">None</option>
                <option value="Arduino/RPi">Arduino/RPi</option>
                <option value="ROS">ROS</option>
                <option value="Robotics Kit">Robotics Kit</option>
              </select>
            </div>
            <div className={styles.formGroup}>
              <label htmlFor="preferredLearning">Preferred Learning Style:</label>
              <select
                id="preferredLearning"
                value={preferredLearning}
                onChange={(e: ChangeEvent<HTMLSelectElement>) => setPreferredLearning(e.target.value as 'Visual' | 'Code-heavy' | 'Theory' | 'Hands-on')}
              >
                <option value="Visual">Visual</option>
                <option value="Code-heavy">Code-heavy</option>
                <option value="Theory">Theory</option>
                <option value="Hands-on">Hands-on</option>
              </select>
            </div>
            <button type="submit" className="button button--primary button--block">Sign Up</button>
            {message && <p className={styles.message}>{message}</p>}
          </form>
        </div>
      </main>
    </Layout>
  );
};

export default Signup;

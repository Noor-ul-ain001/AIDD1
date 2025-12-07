import React from 'react';
import clsx from 'clsx';
import styles from './PersonalizationButton.module.css'; // Assuming a CSS module for styling
import axios from 'axios';

interface PersonalizationButtonProps {
  chapterId: string;
  onPersonalize: (adaptedContent: string) => void;
}

const PersonalizationButton = ({ chapterId, onPersonalize }: PersonalizationButtonProps) => {
  const handlePersonalizeClick = async () => {
    const accessToken = localStorage.getItem('accessToken');
    if (!accessToken) {
      alert('Please log in to personalize content.');
      return;
    }

    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      const response = await axios.post(
        `${backendUrl}/chapters/${chapterId}/personalize`,
        {}, // No request body needed as user profile is derived from token
        {
          headers: {
            Authorization: `Bearer ${accessToken}`,
          },
        }
      );
      onPersonalize(response.data.content);
      alert('Chapter personalized!');
    } catch (error) {
      console.error('Error personalizing chapter:', error);
      alert('Failed to personalize chapter. Please try again or log in.');
    }
  };

  return (
    <button
      className={clsx('button button--primary', styles.personalizeButton)}
      onClick={handlePersonalizeClick}
    >
      Personalize Chapter
    </button>
  );
};

export default PersonalizationButton;

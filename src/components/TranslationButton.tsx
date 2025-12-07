import React from 'react';
import clsx from 'clsx';
import styles from './TranslationButton.module.css'; // Assuming a CSS module for styling
import axios from 'axios';

interface TranslationButtonProps {
  chapterId: string;
  onTranslate: (translatedContent: string) => void;
}

const TranslationButton = ({ chapterId, onTranslate }: TranslationButtonProps) => {
  const handleTranslateClick = async () => {
    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
      const response = await axios.post(`${backendUrl}/chapters/${chapterId}/translate`);
      onTranslate(response.data.content);
      alert('Chapter translated to Urdu!');
    } catch (error) {
      console.error('Error translating chapter:', error);
      alert('Failed to translate chapter. Please try again.');
    }
  };

  return (
    <button
      className={clsx('button button--secondary', styles.translateButton)}
      onClick={handleTranslateClick}
    >
      Translate to Urdu
    </button>
  );
};

export default TranslationButton;

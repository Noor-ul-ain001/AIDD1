import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './FloatingChatbotIcon.module.css'; // Assuming a CSS module for styling
import Chatbot from './Chatbot'; // Import the existing Chatbot component

const FloatingChatbotIcon = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      <button className={clsx(styles.chatbotIcon, { [styles.isOpen]: isOpen })} onClick={toggleChatbot}>
        {isOpen ? 'X' : '💬'} {/* Change icon based on state */}
      </button>

      {isOpen && (
        <div className={styles.chatbotOverlay}>
          <div className={styles.chatbotWindow}>
            <Chatbot />
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbotIcon;

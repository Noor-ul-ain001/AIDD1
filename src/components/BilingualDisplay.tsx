import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './BilingualDisplay.module.css';

interface BilingualDisplayProps {
  englishContent: string;
  urduContent: string;
  title?: string;
  showCopyButtons?: boolean;
  showLineNumbers?: boolean;
}

type DisplayMode = 'english' | 'urdu' | 'side-by-side' | 'bilingual';

const BilingualDisplay = ({ 
  englishContent, 
  urduContent, 
  title = "Bilingual Content",
  showCopyButtons = true,
  showLineNumbers = false 
}: BilingualDisplayProps) => {
  const [displayMode, setDisplayMode] = useState<DisplayMode>('bilingual');
  const [copied, setCopied] = useState<string | null>(null);
  const [fontSize, setFontSize] = useState<number>(16);
  const [lineSpacing, setLineSpacing] = useState<number>(1.5);

  const renderContent = (content: string, language: 'english' | 'urdu') => {
    const lines = content.split('\n');
    
    return (
      <div 
        className={clsx(
          styles.contentText,
          language === 'urdu' && styles.urduText,
          showLineNumbers && styles.withLineNumbers
        )}
        style={{
          fontSize: `${fontSize}px`,
          lineHeight: lineSpacing
        }}
      >
        {showLineNumbers ? (
          <div className={styles.lineNumberedContent}>
            {lines.map((line, index) => (
              <div key={index} className={styles.line}>
                <span className={styles.lineNumber}>{index + 1}</span>
                <span dangerouslySetInnerHTML={{ __html: line || '&nbsp;' }} />
              </div>
            ))}
          </div>
        ) : (
          <div dangerouslySetInnerHTML={{ __html: content }} />
        )}
      </div>
    );
  };

  const handleCopy = (text: string, language: string) => {
    navigator.clipboard.writeText(text)
      .then(() => {
        setCopied(language);
        setTimeout(() => setCopied(null), 2000);
      });
  };

  const displayModes = [
    { id: 'bilingual', label: 'Bilingual', icon: '🌐' },
    { id: 'side-by-side', label: 'Side-by-Side', icon: '📖' },
    { id: 'english', label: 'English Only', icon: '🇬🇧' },
    { id: 'urdu', label: 'Urdu Only', icon: '🇵🇰' },
  ];

  return (
    <div className={styles.bilingualDisplay}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <h2 className={styles.title}>{title}</h2>
          <div className={styles.controlsGroup}>
            <label className={styles.controlLabel}>
              Font Size:
              <div className={styles.fontSizeControls}>
                <button 
                  className={styles.sizeBtn}
                  onClick={() => setFontSize(Math.max(12, fontSize - 2))}
                  disabled={fontSize <= 12}
                >
                  -
                </button>
                <span className={styles.sizeValue}>{fontSize}px</span>
                <button 
                  className={styles.sizeBtn}
                  onClick={() => setFontSize(Math.min(24, fontSize + 2))}
                  disabled={fontSize >= 24}
                >
                  +
                </button>
              </div>
            </label>
            
            <label className={styles.controlLabel}>
              Line Spacing:
              <input
                type="range"
                min="1"
                max="3"
                step="0.1"
                value={lineSpacing}
                onChange={(e) => setLineSpacing(parseFloat(e.target.value))}
                className={styles.spacingSlider}
              />
            </label>
          </div>
        </div>
        
        {showCopyButtons && (
          <div className={styles.copyButtons}>
            <button
              className={clsx(styles.copyBtn, copied === 'english' && styles.copied)}
              onClick={() => handleCopy(englishContent, 'english')}
            >
              {copied === 'english' ? '✓ Copied!' : '📋 Copy English'}
            </button>
            <button
              className={clsx(styles.copyBtn, copied === 'urdu' && styles.copied)}
              onClick={() => handleCopy(urduContent, 'urdu')}
            >
              {copied === 'urdu' ? '✓ Copied!' : '📋 Copy Urdu'}
            </button>
          </div>
        )}
      </div>

      {/* Mode Selector */}
      <div className={styles.modeSelector}>
        {displayModes.map((mode) => (
          <button
            key={mode.id}
            className={clsx(
              styles.modeBtn,
              displayMode === mode.id && styles.modeBtnActive
            )}
            onClick={() => setDisplayMode(mode.id as DisplayMode)}
          >
            <span className={styles.modeIcon}>{mode.icon}</span>
            <span className={styles.modeLabel}>{mode.label}</span>
          </button>
        ))}
      </div>

      {/* Content Area */}
      <div className={styles.contentArea}>
        {/* Bilingual View (Alternating paragraphs) */}
        {displayMode === 'bilingual' && (
          <div className={styles.bilingualContainer}>
            <div className={styles.bilingualContent}>
              {englishContent.split('\n\n').map((engPara, index) => (
                <div key={index} className={styles.bilingualPair}>
                  <div className={clsx(styles.paragraph, styles.englishParagraph)}>
                    {showLineNumbers && (
                      <span className={styles.paraNumber}>{index + 1}</span>
                    )}
                    <div dangerouslySetInnerHTML={{ __html: engPara }} />
                  </div>
                  <div className={clsx(styles.paragraph, styles.urduParagraph)}>
                    <div dangerouslySetInnerHTML={{ __html: urduContent.split('\n\n')[index] || '' }} />
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Side-by-Side View */}
        {displayMode === 'side-by-side' && (
          <div className={styles.sideBySideContainer}>
            <div className={clsx(styles.pane, styles.englishPane)}>
              <div className={styles.paneHeader}>
                <h3 className={styles.paneTitle}>
                  <span className={styles.langIcon}>🇬🇧</span>
                  English Version
                </h3>
                <span className={styles.wordCount}>
                  {englishContent.split(/\s+/).length} words
                </span>
              </div>
              {renderContent(englishContent, 'english')}
            </div>
            
            <div className={styles.divider} />
            
            <div className={clsx(styles.pane, styles.urduPane)}>
              <div className={styles.paneHeader}>
                <h3 className={styles.paneTitle}>
                  <span className={styles.langIcon}>🇵🇰</span>
                  اردو ورژن
                </h3>
                <span className={styles.wordCount}>
                  {urduContent.split(/\s+/).length} الفاظ
                </span>
              </div>
              {renderContent(urduContent, 'urdu')}
            </div>
          </div>
        )}

        {/* English Only View */}
        {displayMode === 'english' && (
          <div className={clsx(styles.singlePane, styles.englishPane)}>
            <div className={styles.paneHeader}>
              <h3 className={styles.paneTitle}>
                <span className={styles.langIcon}>🇬🇧</span>
                English Version
              </h3>
              <span className={styles.wordCount}>
                {englishContent.split(/\s+/).length} words
              </span>
            </div>
            {renderContent(englishContent, 'english')}
          </div>
        )}

        {/* Urdu Only View */}
        {displayMode === 'urdu' && (
          <div className={clsx(styles.singlePane, styles.urduPane)}>
            <div className={styles.paneHeader}>
              <h3 className={styles.paneTitle}>
                <span className={styles.langIcon}>🇵🇰</span>
                اردو ورژن
              </h3>
              <span className={styles.wordCount}>
                {urduContent.split(/\s+/).length} الفاظ
              </span>
            </div>
            {renderContent(urduContent, 'urdu')}
          </div>
        )}
      </div>

      {/* Footer Stats */}
      <div className={styles.footer}>
        <div className={styles.stats}>
          <div className={styles.stat}>
            <span className={styles.statLabel}>English Words:</span>
            <span className={styles.statValue}>{englishContent.split(/\s+/).length}</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statLabel}>Urdu Words:</span>
            <span className={styles.statValue}>{urduContent.split(/\s+/).length}</span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statLabel}>Font Size:</span>
            <span className={styles.statValue}>{fontSize}px</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default BilingualDisplay;
import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import headerStyles from '../components/HomepageHeader.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', headerStyles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">Your journey into the world of robotics starts here.</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/ros2-intro">
                Start Learning Now
              </Link>
            </div>
          </div>
        </div>
        <div className={headerStyles.heroImages}>
          <img src="/img/ai-placeholder-1.svg" alt="AI Placeholder 1" className={clsx(headerStyles.heroImage, 'tech-pulse')} />
          <img src="/img/ai-placeholder-2.svg" alt="AI Placeholder 2" className={clsx(headerStyles.heroImage, 'tech-pulse')} style={{animationDelay: '0.5s'}} />
          <img src="/img/ai-placeholder-3.svg" alt="AI Placeholder 3" className={clsx(headerStyles.heroImage, 'tech-pulse')} style={{animationDelay: '1s'}} />
        </div>
      </div>
    </header>
  );
}

// Technical Features Component
function HomepageFeatures(): ReactNode {
  return (
    <section className={clsx('features-section', styles.features)}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <div className={clsx('feature-icon', styles.featureIcon)}>
                <svg className="tech-icon" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M12 2L2 7L12 12L22 7L12 2Z" stroke="var(--ifm-color-primary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M2 17L12 22L22 17" stroke="var(--ifm-color-primary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M2 12L12 17L22 12" stroke="var(--ifm-color-primary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </div>
              <h3>Advanced Robotics</h3>
              <p>Learn cutting-edge robotics concepts with ROS 2, NVIDIA Isaac, and more.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <div className={clsx('feature-icon', styles.featureIcon)}>
                <svg className="tech-icon" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M13 2L3 14H12L11 22L21 10H12L13 2Z" stroke="var(--ifm-color-secondary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </div>
              <h3>AI Integration</h3>
              <p>Combine robotics with artificial intelligence for next-generation applications.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <div className={clsx('feature-icon', styles.featureIcon)}>
                <svg className="tech-icon" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M12 2C6.48 2 2 6.48 2 12C2 17.52 6.48 22 12 22C17.52 22 22 17.52 12 12Z" stroke="var(--ifm-color-tertiary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  <path d="M12 6V12L16 14" stroke="var(--ifm-color-tertiary)" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </div>
              <h3>Future-Ready</h3>
              <p>Stay ahead with the latest in humanoid robotics and physical AI.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn ROS 2 with our comprehensive tutorials.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

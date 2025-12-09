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
        <div className={headerStyles.heroImages}>
          <img src="/img/ai-placeholder-1.svg" alt="AI Placeholder 1" className={headerStyles.heroImage} />
          <img src="/img/ai-placeholder-2.svg" alt="AI Placeholder 2" className={headerStyles.heroImage} />
          <img src="/img/ai-placeholder-3.svg" alt="AI Placeholder 3" className={headerStyles.heroImage} />
        </div>
      </div>
    </header>
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
        {/* <HomepageFeatures /> */}
      </main>
    </Layout>
  );
}

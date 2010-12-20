

SET SQL_MODE="NO_AUTO_VALUE_ON_ZERO";


--
-- Base de données: `pow_sql`
--
CREATE DATABASE pow_sql;
-- --------------------------------------------------------

GRANT ALL PRIVILEGES ON pow_sql.* TO pow_user@localhost IDENTIFIED BY 'pwdpow_user';

use pow_sql;
--
-- Structure de la table `connexion`
--

CREATE TABLE IF NOT EXISTS `connexion` (
  `webId` int(11) NOT NULL AUTO_INCREMENT,
  `login` varchar(50) NOT NULL,
  `start` date NOT NULL,
  `end` date DEFAULT NULL,
  PRIMARY KEY (`webId`)
) TYPE=INNODB AUTO_INCREMENT=1 ;

--
-- Contenu de la table `connexion`
--


-- --------------------------------------------------------

--
-- Structure de la table `log`
--

CREATE TABLE IF NOT EXISTS `log` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `msg` varchar(300) DEFAULT NULL,
  `webId` int(11) NOT NULL,
  PRIMARY KEY (`id`),
  FOREIGN KEY (`webId`) REFERENCES connexion(webId) ON DELETE CASCADE
) TYPE=INNODB  AUTO_INCREMENT=1 ;

--
-- Contenu de la table `log`
--

